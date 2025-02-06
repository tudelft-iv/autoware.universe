// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__SCENE_MODULE_INTERFACE_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__SCENE_MODULE_INTERFACE_HPP_

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware/motion_utils/factor/velocity_factor_interface.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>
#include <autoware/rtc_interface/rtc_interface.hpp>
#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/ros/parameter.hpp>
#include <autoware/universe_utils/system/time_keeper.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <autoware_adapi_v1_msgs/msg/velocity_factor.hpp>
#include <autoware_adapi_v1_msgs/msg/velocity_factor_array.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_rtc_msgs/msg/state.hpp>
#include <tier4_v2x_msgs/msg/infrastructure_command_array.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Debug
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
namespace autoware::behavior_velocity_planner
{

using autoware::motion_utils::PlanningBehavior;
using autoware::motion_utils::VelocityFactor;
using autoware::objects_of_interest_marker_interface::ColorName;
using autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface;
using autoware::rtc_interface::RTCInterface;
using autoware::universe_utils::DebugPublisher;
using autoware::universe_utils::getOrDeclareParameter;
using builtin_interfaces::msg::Time;
using tier4_debug_msgs::msg::Float64Stamped;
using tier4_planning_msgs::msg::PathWithLaneId;
using tier4_rtc_msgs::msg::Module;
using tier4_rtc_msgs::msg::State;
using unique_identifier_msgs::msg::UUID;

struct ObjectOfInterest
{
  geometry_msgs::msg::Pose pose;
  autoware_perception_msgs::msg::Shape shape;
  ColorName color;
  ObjectOfInterest(
    const geometry_msgs::msg::Pose & pose, autoware_perception_msgs::msg::Shape shape,
    const ColorName & color_name)
  : pose(pose), shape(std::move(shape)), color(color_name)
  {
  }
};

class SceneModuleInterface
{
public:
  explicit SceneModuleInterface(
    const int64_t module_id, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);
  virtual ~SceneModuleInterface() = default;

  virtual bool modifyPathVelocity(PathWithLaneId * path) = 0;

  virtual visualization_msgs::msg::MarkerArray createDebugMarkerArray() = 0;
  virtual std::vector<autoware::motion_utils::VirtualWall> createVirtualWalls() = 0;

  int64_t getModuleId() const { return module_id_; }

  void setPlannerData(const std::shared_ptr<const PlannerData> & planner_data)
  {
    planner_data_ = planner_data;
  }

  std::optional<tier4_v2x_msgs::msg::InfrastructureCommand> getInfrastructureCommand()
  {
    return infrastructure_command_;
  }

  void setInfrastructureCommand(
    const std::optional<tier4_v2x_msgs::msg::InfrastructureCommand> & command)
  {
    infrastructure_command_ = command;
  }

  void setTimeKeeper(const std::shared_ptr<universe_utils::TimeKeeper> & time_keeper)
  {
    time_keeper_ = time_keeper;
  }

  std::shared_ptr<universe_utils::TimeKeeper> getTimeKeeper() { return time_keeper_; }

  void setActivation(const bool activated) { activated_ = activated; }
  void setRTCEnabled(const bool enable_rtc) { rtc_enabled_ = enable_rtc; }
  bool isActivated() const { return activated_; }
  bool isSafe() const { return safe_; }
  double getDistance() const { return distance_; }

  void resetVelocityFactor() { velocity_factor_.reset(); }
  VelocityFactor getVelocityFactor() const { return velocity_factor_.get(); }
  std::vector<ObjectOfInterest> getObjectsOfInterestData() const { return objects_of_interest_; }
  void clearObjectsOfInterestData() { objects_of_interest_.clear(); }

protected:
  const int64_t module_id_;
  bool activated_;
  bool safe_;
  bool rtc_enabled_;
  double distance_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<const PlannerData> planner_data_;
  std::optional<tier4_v2x_msgs::msg::InfrastructureCommand> infrastructure_command_;
  autoware::motion_utils::VelocityFactorInterface velocity_factor_;
  std::vector<ObjectOfInterest> objects_of_interest_;
  mutable std::shared_ptr<universe_utils::TimeKeeper> time_keeper_;

  void setSafe(const bool safe)
  {
    safe_ = safe;
    if (!rtc_enabled_) {
      syncActivation();
    }
  }
  void setDistance(const double distance) { distance_ = distance; }
  void syncActivation() { setActivation(isSafe()); }

  void setObjectsOfInterestData(
    const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape,
    const ColorName & color_name)
  {
    objects_of_interest_.emplace_back(pose, shape, color_name);
  }

  size_t findEgoSegmentIndex(
    const std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> & points) const;
};

class SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterface(rclcpp::Node & node, [[maybe_unused]] const char * module_name);

  virtual ~SceneModuleManagerInterface() = default;

  virtual const char * getModuleName() = 0;

  void updateSceneModuleInstances(
    const std::shared_ptr<const PlannerData> & planner_data,
    const tier4_planning_msgs::msg::PathWithLaneId & path);

  virtual void plan(tier4_planning_msgs::msg::PathWithLaneId * path) { modifyPathVelocity(path); }

protected:
  virtual void modifyPathVelocity(tier4_planning_msgs::msg::PathWithLaneId * path);

  virtual void launchNewModules(const tier4_planning_msgs::msg::PathWithLaneId & path) = 0;

  virtual std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
  getModuleExpiredFunction(const tier4_planning_msgs::msg::PathWithLaneId & path) = 0;

  virtual void deleteExpiredModules(const tier4_planning_msgs::msg::PathWithLaneId & path);

  bool isModuleRegistered(const int64_t module_id)
  {
    return registered_module_id_set_.count(module_id) != 0;
  }

  void registerModule(const std::shared_ptr<SceneModuleInterface> & scene_module);

  size_t findEgoSegmentIndex(
    const std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> & points) const;

  std::set<std::shared_ptr<SceneModuleInterface>> scene_modules_;
  std::set<int64_t> registered_module_id_set_;

  std::shared_ptr<const PlannerData> planner_data_;
  autoware::motion_utils::VirtualWallMarkerCreator virtual_wall_marker_creator_;

  rclcpp::Node & node_;
  rclcpp::Clock::SharedPtr clock_;
  // Debug
  bool is_publish_debug_path_ = {false};  // note : this is very heavy debug topic option
  rclcpp::Logger logger_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_virtual_wall_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_;
  rclcpp::Publisher<tier4_planning_msgs::msg::PathWithLaneId>::SharedPtr pub_debug_path_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::VelocityFactorArray>::SharedPtr
    pub_velocity_factor_;
  rclcpp::Publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>::SharedPtr
    pub_infrastructure_commands_;

  std::shared_ptr<DebugPublisher> processing_time_publisher_;

  rclcpp::Publisher<universe_utils::ProcessingTimeDetail>::SharedPtr pub_processing_time_detail_;

  std::shared_ptr<universe_utils::TimeKeeper> time_keeper_;
};

class SceneModuleManagerInterfaceWithRTC : public SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterfaceWithRTC(
    rclcpp::Node & node, const char * module_name, const bool enable_rtc = true);

  void plan(tier4_planning_msgs::msg::PathWithLaneId * path) override;

protected:
  RTCInterface rtc_interface_;
  std::unordered_map<int64_t, UUID> map_uuid_;

  ObjectsOfInterestMarkerInterface objects_of_interest_marker_interface_;

  virtual void sendRTC(const Time & stamp);

  virtual void setActivation();

  void updateRTCStatus(
    const UUID & uuid, const bool safe, const uint8_t state, const double distance,
    const Time & stamp)
  {
    rtc_interface_.updateCooperateStatus(uuid, safe, state, distance, distance, stamp);
  }

  void removeRTCStatus(const UUID & uuid) { rtc_interface_.removeCooperateStatus(uuid); }

  void publishRTCStatus(const Time & stamp)
  {
    rtc_interface_.removeExpiredCooperateStatus();
    rtc_interface_.publishCooperateStatus(stamp);
  }

  UUID getUUID(const int64_t & module_id) const;

  void generateUUID(const int64_t & module_id);

  void removeUUID(const int64_t & module_id);

  void publishObjectsOfInterestMarker();

  void deleteExpiredModules(const tier4_planning_msgs::msg::PathWithLaneId & path) override;

  static bool getEnableRTC(rclcpp::Node & node, const std::string & param_name)
  {
    bool enable_rtc = true;

    try {
      enable_rtc = getOrDeclareParameter<bool>(node, "enable_all_modules_auto_mode")
                     ? false
                     : getOrDeclareParameter<bool>(node, param_name);
    } catch (const std::exception & e) {
      enable_rtc = getOrDeclareParameter<bool>(node, param_name);
    }

    return enable_rtc;
  }
};

}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__SCENE_MODULE_INTERFACE_HPP_
