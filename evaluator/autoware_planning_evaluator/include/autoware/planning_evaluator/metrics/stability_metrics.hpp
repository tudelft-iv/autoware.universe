// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__PLANNING_EVALUATOR__METRICS__STABILITY_METRICS_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__METRICS__STABILITY_METRICS_HPP_

#include "autoware/universe_utils/math/accumulator.hpp"

#include "autoware_planning_msgs/msg/trajectory.hpp"

namespace planning_diagnostics
{
namespace metrics
{
using autoware::universe_utils::Accumulator;
using autoware_planning_msgs::msg::Trajectory;

/**
 * @brief calculate the discrete Frechet distance between two trajectories
 * @param [in] traj1 first trajectory
 * @param [in] traj2 second trajectory
 * @return calculated statistics
 */
Accumulator<double> calcFrechetDistance(const Trajectory & traj1, const Trajectory & traj2);

/**
 * @brief calculate the lateral distance between two trajectories
 * @param [in] traj1 first trajectory
 * @param [in] traj2 second trajectory
 * @return calculated statistics
 */
Accumulator<double> calcLateralDistance(const Trajectory & traj1, const Trajectory & traj2);

}  // namespace metrics
}  // namespace planning_diagnostics

#endif  // AUTOWARE__PLANNING_EVALUATOR__METRICS__STABILITY_METRICS_HPP_
