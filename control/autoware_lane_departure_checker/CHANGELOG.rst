^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_lane_departure_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* test(lane_departure_checker): add tests for createVehicleFootprints (`#8928 <https://github.com/autowarefoundation/autoware.universe/issues/8928>`_)
  * move createVehicleFootprints() to seperate files
  * add tests for createVehicleFootprints()
  ---------
* test(lane_departure_checker): add tests for resampleTrajectory (`#8895 <https://github.com/autowarefoundation/autoware.universe/issues/8895>`_)
  * move resampleTrajectory() to separate file
  * add tests for resampleTrajectory()
  ---------
* test(lane_departure_checker): add tests for cutTrajectory (`#8887 <https://github.com/autowarefoundation/autoware.universe/issues/8887>`_)
  * move cutTrajectory() to separate file
  * add test for cutTrajectory()
  ---------
* fix(control): align the parameters with launcher (`#8789 <https://github.com/autowarefoundation/autoware.universe/issues/8789>`_)
  align the control parameters
* refactor(start_planner, lane_departure_checker): remove redundant calculation in fuseLaneletPolygon (`#8682 <https://github.com/autowarefoundation/autoware.universe/issues/8682>`_)
  * remove redundant fused lanelet calculation
  * remove unnecessary change
  * add new function
  * fix spelling mistake
  * fix spelling mistake
  * use std::move and lambda funcion for better code
  * add comment for better understanding
  * fix cppcheck
  ---------
* fix(autoware_lane_departure_checker): not to show error message "trajectory deviation is too large" during manual driving (`#8404 <https://github.com/autowarefoundation/autoware.universe/issues/8404>`_)
  * update: update not to show error message "trajectory deviation is too large" during manual driving
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(lane_departure_checker): fix uninitialized variables (`#8451 <https://github.com/autowarefoundation/autoware.universe/issues/8451>`_)
  fix(lane_departure_checker): fix uninitialized_variables
* feat(start_planner): add time_keeper (`#8254 <https://github.com/autowarefoundation/autoware.universe/issues/8254>`_)
  * feat(start_planner): add time_keeper
  * fix
  * fix
  * fix shadow variables
  ---------
* fix(autoware_lane_departure_checker): fix shadowVariable (`#7931 <https://github.com/autowarefoundation/autoware.universe/issues/7931>`_)
  fix:shadowVariable
* refactor(autoware_universe_utils): changed the API to be more intuitive and added documentation (`#7443 <https://github.com/autowarefoundation/autoware.universe/issues/7443>`_)
  * refactor(tier4_autoware_utils): Changed the API to be more intuitive and added documentation.
  * use raw shared ptr in PollingPolicy::NEWEST
  * update
  * fix
  * Update evaluator/autoware_control_evaluator/include/autoware/control_evaluator/control_evaluator_node.hpp
  Co-authored-by: danielsanchezaran <daniel.sanchez@tier4.jp>
  ---------
  Co-authored-by: danielsanchezaran <daniel.sanchez@tier4.jp>
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware.universe/issues/7640>`_)
* feat(motion_velocity_planner, lane_departure_checker): add processing time Float64 publishers (`#7683 <https://github.com/autowarefoundation/autoware.universe/issues/7683>`_)
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware.universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware.universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware.universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(control)!: refactor directory structures of the control checkers (`#7524 <https://github.com/autowarefoundation/autoware.universe/issues/7524>`_)
  * aeb
  * control_validator
  * lane_departure_checker
  * shift_decider
  * fix
  ---------
* refactor(autoware_lane_departure_checker)!: rename directory name  (`#7410 <https://github.com/autowarefoundation/autoware.universe/issues/7410>`_)
* Contributors: Go Sakayori, Kosuke Takeuchi, Kyoichi Sugahara, Maxime CLEMENT, Mitsuhiro Sakamoto, T-Kimura-MM, Takayuki Murooka, Yuki TAKAGI, Yukinari Hisaki, Yutaka Kondo, Zhe Shen, kobayu858

0.26.0 (2024-04-03)
-------------------
