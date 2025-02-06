// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <autoware/universe_utils/system/time_keeper.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{

using autoware::universe_utils::StopWatch;

SceneModuleInterface::SceneModuleInterface(
  const int64_t module_id, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: module_id_(module_id),
  activated_(false),
  safe_(false),
  distance_(std::numeric_limits<double>::lowest()),
  logger_(logger),
  clock_(clock),
  time_keeper_(std::shared_ptr<universe_utils::TimeKeeper>())
{
}

size_t SceneModuleInterface::findEgoSegmentIndex(
  const std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> & points) const
{
  const auto & p = planner_data_;
  return autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    points, p->current_odometry->pose, p->ego_nearest_dist_threshold);
}

SceneModuleManagerInterface::SceneModuleManagerInterface(
  rclcpp::Node & node, [[maybe_unused]] const char * module_name)
: node_(node), clock_(node.get_clock()), logger_(node.get_logger())
{
  const auto ns = std::string("~/debug/") + module_name;
  pub_debug_ = node.create_publisher<visualization_msgs::msg::MarkerArray>(ns, 1);
  if (!node.has_parameter("is_publish_debug_path")) {
    is_publish_debug_path_ = node.declare_parameter<bool>("is_publish_debug_path");
  } else {
    is_publish_debug_path_ = node.get_parameter("is_publish_debug_path").as_bool();
  }
  if (is_publish_debug_path_) {
    pub_debug_path_ = node.create_publisher<tier4_planning_msgs::msg::PathWithLaneId>(
      std::string("~/debug/path_with_lane_id/") + module_name, 1);
  }
  pub_virtual_wall_ = node.create_publisher<visualization_msgs::msg::MarkerArray>(
    std::string("~/virtual_wall/") + module_name, 5);
  pub_velocity_factor_ = node.create_publisher<autoware_adapi_v1_msgs::msg::VelocityFactorArray>(
    std::string("/planning/velocity_factors/") + module_name, 1);
  pub_infrastructure_commands_ =
    node.create_publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>(
      "~/output/infrastructure_commands", 1);

  processing_time_publisher_ = std::make_shared<DebugPublisher>(&node, "~/debug");

  pub_processing_time_detail_ = node.create_publisher<universe_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/" + std::string(module_name), 1);

  time_keeper_ = std::make_shared<universe_utils::TimeKeeper>(pub_processing_time_detail_);
}

size_t SceneModuleManagerInterface::findEgoSegmentIndex(
  const std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> & points) const
{
  const auto & p = planner_data_;
  return autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    points, p->current_odometry->pose, p->ego_nearest_dist_threshold, p->ego_nearest_yaw_threshold);
}

void SceneModuleManagerInterface::updateSceneModuleInstances(
  const std::shared_ptr<const PlannerData> & planner_data,
  const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  planner_data_ = planner_data;

  launchNewModules(path);
  deleteExpiredModules(path);
}

void SceneModuleManagerInterface::modifyPathVelocity(
  tier4_planning_msgs::msg::PathWithLaneId * path)
{
  universe_utils::ScopedTimeTrack st(
    "SceneModuleManagerInterface::modifyPathVelocity", *time_keeper_);
  StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("Total");
  visualization_msgs::msg::MarkerArray debug_marker_array;
  autoware_adapi_v1_msgs::msg::VelocityFactorArray velocity_factor_array;
  velocity_factor_array.header.frame_id = "map";
  velocity_factor_array.header.stamp = clock_->now();

  tier4_v2x_msgs::msg::InfrastructureCommandArray infrastructure_command_array;
  infrastructure_command_array.stamp = clock_->now();

  for (const auto & scene_module : scene_modules_) {
    scene_module->resetVelocityFactor();
    scene_module->setPlannerData(planner_data_);
    scene_module->modifyPathVelocity(path);

    // The velocity factor must be called after modifyPathVelocity.
    const auto velocity_factor = scene_module->getVelocityFactor();
    if (velocity_factor.behavior != PlanningBehavior::UNKNOWN) {
      velocity_factor_array.factors.emplace_back(velocity_factor);
    }

    if (const auto command = scene_module->getInfrastructureCommand()) {
      infrastructure_command_array.commands.push_back(*command);
    }

    for (const auto & marker : scene_module->createDebugMarkerArray().markers) {
      debug_marker_array.markers.push_back(marker);
    }

    virtual_wall_marker_creator_.add_virtual_walls(scene_module->createVirtualWalls());
  }

  pub_velocity_factor_->publish(velocity_factor_array);
  pub_infrastructure_commands_->publish(infrastructure_command_array);
  pub_debug_->publish(debug_marker_array);
  if (is_publish_debug_path_) {
    tier4_planning_msgs::msg::PathWithLaneId debug_path;
    debug_path.header = path->header;
    debug_path.points = path->points;
    pub_debug_path_->publish(debug_path);
  }
  pub_virtual_wall_->publish(virtual_wall_marker_creator_.create_markers(clock_->now()));
  processing_time_publisher_->publish<Float64Stamped>(
    std::string(getModuleName()) + "/processing_time_ms", stop_watch.toc("Total"));
}

void SceneModuleManagerInterface::deleteExpiredModules(
  const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  const auto isModuleExpired = getModuleExpiredFunction(path);

  auto itr = scene_modules_.begin();
  while (itr != scene_modules_.end()) {
    if (isModuleExpired(*itr)) {
      itr = scene_modules_.erase(itr);
    } else {
      itr++;
    }
  }
}

void SceneModuleManagerInterface::registerModule(
  const std::shared_ptr<SceneModuleInterface> & scene_module)
{
  RCLCPP_DEBUG(
    logger_, "register task: module = %s, id = %lu", getModuleName(), scene_module->getModuleId());
  registered_module_id_set_.emplace(scene_module->getModuleId());
  scene_module->setTimeKeeper(time_keeper_);
  scene_modules_.insert(scene_module);
}

SceneModuleManagerInterfaceWithRTC::SceneModuleManagerInterfaceWithRTC(
  rclcpp::Node & node, const char * module_name, const bool enable_rtc)
: SceneModuleManagerInterface(node, module_name),
  rtc_interface_(&node, module_name, enable_rtc),
  objects_of_interest_marker_interface_(&node, module_name)
{
}

void SceneModuleManagerInterfaceWithRTC::plan(tier4_planning_msgs::msg::PathWithLaneId * path)
{
  setActivation();
  modifyPathVelocity(path);
  sendRTC(path->header.stamp);
  publishObjectsOfInterestMarker();
}

void SceneModuleManagerInterfaceWithRTC::sendRTC(const Time & stamp)
{
  for (const auto & scene_module : scene_modules_) {
    const UUID uuid = getUUID(scene_module->getModuleId());
    const auto state = !scene_module->isActivated() && scene_module->isSafe()
                         ? State::WAITING_FOR_EXECUTION
                         : State::RUNNING;
    updateRTCStatus(uuid, scene_module->isSafe(), state, scene_module->getDistance(), stamp);
  }
  publishRTCStatus(stamp);
}

void SceneModuleManagerInterfaceWithRTC::setActivation()
{
  for (const auto & scene_module : scene_modules_) {
    const UUID uuid = getUUID(scene_module->getModuleId());
    scene_module->setActivation(rtc_interface_.isActivated(uuid));
    scene_module->setRTCEnabled(rtc_interface_.isRTCEnabled(uuid));
  }
}

UUID SceneModuleManagerInterfaceWithRTC::getUUID(const int64_t & module_id) const
{
  if (map_uuid_.count(module_id) == 0) {
    const UUID uuid;
    return uuid;
  }
  return map_uuid_.at(module_id);
}

void SceneModuleManagerInterfaceWithRTC::generateUUID(const int64_t & module_id)
{
  map_uuid_.insert({module_id, autoware::universe_utils::generateUUID()});
}

void SceneModuleManagerInterfaceWithRTC::removeUUID(const int64_t & module_id)
{
  const auto result = map_uuid_.erase(module_id);
  if (result == 0) {
    RCLCPP_WARN_STREAM(logger_, "[removeUUID] module_id = " << module_id << " is not registered.");
  }
}

void SceneModuleManagerInterfaceWithRTC::publishObjectsOfInterestMarker()
{
  for (const auto & scene_module : scene_modules_) {
    const auto objects = scene_module->getObjectsOfInterestData();
    for (const auto & obj : objects) {
      objects_of_interest_marker_interface_.insertObjectData(obj.pose, obj.shape, obj.color);
    }
    scene_module->clearObjectsOfInterestData();
  }

  objects_of_interest_marker_interface_.publishMarkerArray();
}

void SceneModuleManagerInterfaceWithRTC::deleteExpiredModules(
  const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  const auto isModuleExpired = getModuleExpiredFunction(path);

  auto itr = scene_modules_.begin();
  while (itr != scene_modules_.end()) {
    if (isModuleExpired(*itr)) {
      const UUID uuid = getUUID((*itr)->getModuleId());
      updateRTCStatus(
        uuid, (*itr)->isSafe(), State::SUCCEEDED, std::numeric_limits<double>::lowest(),
        clock_->now());
      removeUUID((*itr)->getModuleId());
      registered_module_id_set_.erase((*itr)->getModuleId());
      itr = scene_modules_.erase(itr);
    } else {
      itr++;
    }
  }
}

}  // namespace autoware::behavior_velocity_planner
