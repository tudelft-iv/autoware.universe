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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_SAFETY_CHECKER__SAFETY_CHECK_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_SAFETY_CHECKER__SAFETY_CHECK_HPP_

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::utils::path_safety_checker
{

using autoware::behavior_path_planner::utils::path_safety_checker::CollisionCheckDebug;
using autoware::universe_utils::calcYawDeviation;
using autoware::universe_utils::Point2d;
using autoware::universe_utils::Polygon2d;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::Shape;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;

/**
 * @brief Checks if the object is coming toward the ego vehicle judging by yaw deviation
 * @param vehicle_pose Ego vehicle pose.
 * @param object_pose Object pose.
 * @param angle_threshold Angle threshold.
 * @return True if the object vehicle is coming towards the ego vehicle
 */
bool isTargetObjectOncoming(
  const geometry_msgs::msg::Pose & vehicle_pose, const geometry_msgs::msg::Pose & object_pose,
  const double angle_threshold = M_PI_2);

/**
 * @brief Checks if the object is in front of the ego vehicle.
 * @param ego_pose Ego vehicle pose.
 * @param obj_polygon Polygon of object.
 * @param base_to_front Base link to vehicle front.
 * @return True if object is in front.
 */
bool isTargetObjectFront(
  const geometry_msgs::msg::Pose & ego_pose, const Polygon2d & obj_polygon,
  const double base_to_front);

Polygon2d createExtendedPolygon(
  const Pose & base_link_pose, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double lon_length, const double lat_margin, const bool is_stopped_obj,
  CollisionCheckDebug & debug);

Polygon2d createExtendedPolygon(
  const PoseWithVelocityAndPolygonStamped & obj_pose_with_poly, const double lon_length,
  const double lat_margin, const bool is_stopped_obj, CollisionCheckDebug & debug);

PredictedPath convertToPredictedPath(
  const std::vector<PoseWithVelocityStamped> & path, const double time_resolution);

double calcRssDistance(
  const double front_object_velocity, const double rear_object_velocity,
  const RSSparams & rss_params);

/**
 * @brief Calculates the minimum longitudinal length using rss parameter. The object is either ego
 * vehicle or target object.
 * @param front_object_velocity Front object velocity.
 * @param rear_object_velocity Front object velocity.
 * @param RSSparams RSS parameters
 * @return Maximum distance.
 */
double calc_minimum_longitudinal_length(
  const double front_object_velocity, const double rear_object_velocity,
  const RSSparams & rss_params);

/**
 * @brief Calculates an interpolated pose with velocity for a given relative time along a path.
 * @param path A vector of PoseWithVelocityStamped objects representing the path.
 * @param relative_time The relative time at which to calculate the interpolated pose and velocity.
 * @return An optional PoseWithVelocityStamped object. If the interpolation is successful,
 *         it contains the interpolated pose, velocity, and time. If the interpolation fails
 *         (e.g., empty path, negative time, or time beyond the path), it returns std::nullopt.
 */
std::optional<PoseWithVelocityStamped> calc_interpolated_pose_with_velocity(
  const std::vector<PoseWithVelocityStamped> & path, const double relative_time);

std::optional<PoseWithVelocityAndPolygonStamped>
get_interpolated_pose_with_velocity_and_polygon_stamped(
  const std::vector<PoseWithVelocityStamped> & pred_path, const double current_time,
  const VehicleInfo & ego_info);

std::optional<PoseWithVelocityAndPolygonStamped>
get_interpolated_pose_with_velocity_and_polygon_stamped(
  const std::vector<PoseWithVelocityStamped> & pred_path, const double current_time,
  const Shape & shape);

template <typename T, typename F>
std::vector<T> filterPredictedPathByTimeHorizon(
  const std::vector<T> & path, const double time_horizon, const F & interpolateFunc);
std::vector<PoseWithVelocityStamped> filterPredictedPathByTimeHorizon(
  const std::vector<PoseWithVelocityStamped> & path, const double time_horizon);
ExtendedPredictedObject filterObjectPredictedPathByTimeHorizon(
  const ExtendedPredictedObject & object, const double time_horizon,
  const bool check_all_predicted_path);
ExtendedPredictedObjects filterObjectPredictedPathByTimeHorizon(
  const ExtendedPredictedObjects & objects, const double time_horizon,
  const bool check_all_predicted_path);

std::vector<PoseWithVelocityStamped> filterPredictedPathAfterTargetPose(
  const std::vector<PoseWithVelocityStamped> & path, const Pose & target_pose);

bool checkSafetyWithRSS(
  const PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path,
  const std::vector<ExtendedPredictedObject> & objects, CollisionCheckDebugMap & debug_map,
  const BehaviorPathPlannerParameters & parameters, const RSSparams & rss_params,
  const bool check_all_predicted_path, const double hysteresis_factor,
  const double yaw_difference_th);

/**
 * @brief Iterate the points in the ego and target's predicted path and
 *        perform safety check for each of the iterated points.
 * @param planned_path The predicted path of the ego vehicle.
 * @param predicted_ego_path Ego vehicle's predicted path
 * @param ego_current_velocity Current velocity of the ego vehicle.
 * @param target_object The predicted object to check collision with.
 * @param target_object_path The predicted path of the target object.
 * @param common_parameters The common parameters used in behavior path planner.
 * @param front_object_deceleration The deceleration of the object in the front.(used in RSS)
 * @param rear_object_deceleration The deceleration of the object in the rear.(used in RSS)
 * @param yaw_difference_th maximum yaw difference between any given ego path pose and object
 * predicted path pose.
 * @param debug The debug information for collision checking.
 * @return true if distance is safe.
 */
bool checkCollision(
  const PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & predicted_ego_path,
  const ExtendedPredictedObject & target_object,
  const PredictedPathWithPolygon & target_object_path,
  const BehaviorPathPlannerParameters & common_parameters, const RSSparams & rss_parameters,
  const double hysteresis_factor, const double yaw_difference_th, CollisionCheckDebug & debug);

/**
 * @brief Iterate the points in the ego and target's predicted path and
 *        perform safety check for each of the iterated points.
 * @param planned_path The predicted path of the ego vehicle.
 * @param predicted_ego_path Ego vehicle's predicted path
 * @param ego_current_velocity Current velocity of the ego vehicle.
 * @param target_object The predicted object to check collision with.
 * @param target_object_path The predicted path of the target object.
 * @param common_parameters The common parameters used in behavior path planner.
 * @param front_object_deceleration The deceleration of the object in the front.(used in RSS)
 * @param rear_object_deceleration The deceleration of the object in the rear.(used in RSS)
 * @param debug The debug information for collision checking.
 * @return a list of polygon when collision is expected.
 */
std::vector<Polygon2d> get_collided_polygons(
  const PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & predicted_ego_path,
  const ExtendedPredictedObject & target_object,
  const PredictedPathWithPolygon & target_object_path, const VehicleInfo & vehicle_info,
  const RSSparams & rss_parameters, const double hysteresis_factor, const double max_velocity_limit,
  const double yaw_difference_th, CollisionCheckDebug & debug);

bool checkPolygonsIntersects(
  const std::vector<Polygon2d> & polys_1, const std::vector<Polygon2d> & polys_2);
bool checkSafetyWithIntegralPredictedPolygon(
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path, const VehicleInfo & vehicle_info,
  const ExtendedPredictedObjects & objects, const bool check_all_predicted_path,
  const IntegralPredictedPolygonParams & params, CollisionCheckDebugMap & debug_map);

/**
 * @brief Calculates the minimum length from obstacle centroid to outer point.
 * @param shape Object shape.
 * @return Minimum distance.
 */
double calc_obstacle_min_length(const Shape & shape);

/**
 * @brief Calculates the maximum length from obstacle centroid to outer point.
 * @param shape Object shape.
 * @return Maximum distance.
 */
double calc_obstacle_max_length(const Shape & shape);

/**
 * @brief Calculate collision roughly by comparing minimum/maximum distance with margin.
 * @param path The path of the ego vehicle.
 * @param objects The predicted objects.
 * @param margin Distance margin to judge collision.
 * @param parameters The common parameters used in behavior path planner.
 * @param use_offset_ego_point If true, the closest point to the object is calculated by
 * interpolating the path points.
 * @return Collision (rough) between minimum distance and maximum distance
 */
std::pair<bool, bool> checkObjectsCollisionRough(
  const PathWithLaneId & path, const PredictedObjects & objects, const double margin,
  const BehaviorPathPlannerParameters & parameters, const bool use_offset_ego_point);

/**
 * @brief Calculate the rough distance between the ego vehicle and the objects.
 * @param path The path of the ego vehicle.
 * @param objects The predicted objects.
 * @param parameters The common parameters used in behavior path planner.
 * @param use_offset_ego_point If true, the closest point to the object is calculated by
 * interpolating the path points.
 * @param distance_type The type of distance to calculate. "min" or "max". Calculate the distance
 * when the distance is minimized or maximized when the direction of the ego and the object is
 * changed.
 * @return The rough distance between the ego vehicle and the objects.
 */
double calculateRoughDistanceToObjects(
  const PathWithLaneId & path, const PredictedObjects & objects,
  const BehaviorPathPlannerParameters & parameters, const bool use_offset_ego_point,
  const std::string & distance_type);
// debug
CollisionCheckDebugPair createObjectDebug(const ExtendedPredictedObject & obj);
void updateCollisionCheckDebugMap(
  CollisionCheckDebugMap & debug_map, CollisionCheckDebugPair & object_debug, bool is_safe);
}  // namespace autoware::behavior_path_planner::utils::path_safety_checker

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_SAFETY_CHECKER__SAFETY_CHECK_HPP_
