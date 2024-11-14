^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_multi_object_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(object_recognition_utils): add autoware prefix to object_recognition_utils (`#8946 <https://github.com/autowarefoundation/autoware.universe/issues/8946>`_)
* feat(autoware_multi_object_tracker): Set maximum reverse velocity to bicycle and crtv motion models (`#9019 <https://github.com/autowarefoundation/autoware.universe/issues/9019>`_)
  * feat: Add maximum reverse velocity to bicycle and CTRV motion models
  revert the tracker orientation when the velocity exceed the maximum reverse velocity
  refactor: Update motion model parameters for bicycle and CTRV motion models
  * refactor:  check the max_reverse_vel configuration is correct
  max_reverse_vel is expected to be  negative
  * refactor: remove config checker in the initializer
  ---------
* refactor(autoware_multi_object_tracker): separate detected object covariance modeling (`#9001 <https://github.com/autowarefoundation/autoware.universe/issues/9001>`_)
  * refactor: update object model includes in tracker models
  * feat: add uncertainty processor for object tracking
  feat: refactor uncertainty processing for object tracking
  feat: impl obj class model
  feat: Update object model measurement covariances
  Refactor the object model measurement covariances in the `object_model.hpp` file. Update the velocity long and velocity lat measurement covariances for different object model types.
  refactor: Model object uncertainty in multi_object_tracker_node.cpp
  feat: Update object model measurement covariances in object_model.hpp
  feat: Update uncertainty processing for object tracking
  fix: remove uncertainty modelling in trackers
  refactor: Remove unused function isLargeVehicleLabel
  The function isLargeVehicleLabel in utils.hpp is no longer used and can be safely removed.
  Revert "refactor: Remove unused function isLargeVehicleLabel"
  This reverts commit 23e3eff511b21ef8ceeacb7db47c74f747009a32.
  feat: Normalize uncertainty in object tracking
  This commit adds a new function `normalizeUncertainty` to the `uncertainty_processor.hpp` and `uncertainty_processor.cpp` files. The function normalizes the position and twist covariance matrices of detected objects to ensure minimum values for distance, radius, and velocity. This helps improve the accuracy and reliability of object tracking.
  * refactor: update motion model parameters for object tracking
  * refactor: update yaw rate limit in object model
  * Revert "refactor: update yaw rate limit in object model"
  This reverts commit 6e8b201582cb65673678029dc3a781f2b7126f81.
  * refactor: update object model measurement covariances
  Refactor the object model measurement covariances in the `object_model.hpp` file. Update the velocity long and velocity lat measurement covariances for different object model types.
  * refactor: update motion model parameters comments
  * refactor: remove comment
  * style(pre-commit): autofix
  * feat: Update copyright notice in uncertainty_processor.hpp
  Update the copyright notice in the uncertainty_processor.hpp file to reflect the correct company name.
  * refactor: update runProcess function parameters in multi_object_tracker_node.hpp
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_multi_object_tracker): update yaw with range-limited innovation (`#8976 <https://github.com/autowarefoundation/autoware.universe/issues/8976>`_)
  fix: update yaw with range-limited innovation
* feat(autoware_multi_object_tracker): reduce trigger latency (`#8657 <https://github.com/autowarefoundation/autoware.universe/issues/8657>`_)
  * feat: timer-based trigger with phase compensation
  * chore: update comments, name of variable
  * chore: declare min and max publish interval ratios
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_multi_object_tracker): output from screen to both (`#8407 <https://github.com/autowarefoundation/autoware.universe/issues/8407>`_)
* fix(autoware_multi_object_tracker): fix unusedFunction (`#8573 <https://github.com/autowarefoundation/autoware.universe/issues/8573>`_)
  fix:unusedFunction
* chore(autoware_multi_object_tracker): fix typo in input_channels.schema.json (`#8515 <https://github.com/autowarefoundation/autoware.universe/issues/8515>`_)
  * fix(schema): fix typo in input_channels.schema.json
  Fixed a typo in the "lidar_pointpainting" key in the input_channels.schema.json file.
  * fix: fix typo in lidar_pointpainting key
  * chore: fix typo of lidar_pointpainitng channel
  ---------
  Co-authored-by: Shintaro Tomie <58775300+Shin-kyoto@users.noreply.github.com>
* refactor(kalman_filter): prefix package and namespace with autoware (`#7787 <https://github.com/autowarefoundation/autoware.universe/issues/7787>`_)
  * refactor(kalman_filter): prefix package and namespace with autoware
  * move headers to include/autoware/
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* docs(autoware_multi_object_tracker): update input_channels schema with default values (`#8473 <https://github.com/autowarefoundation/autoware.universe/issues/8473>`_)
  chore(perception): update input_channels schema with default values
* fix(autoware_multi_object_tracker): enable trigger publish when delay_compensation is false (`#8484 <https://github.com/autowarefoundation/autoware.universe/issues/8484>`_)
  fix: enable trigger publish when delay_compensation is false
* fix(autoware_multi_object_tracker): fix functionConst (`#8424 <https://github.com/autowarefoundation/autoware.universe/issues/8424>`_)
  fix:functionConst
* docs(autoware_multi_object_tracker): add default values on the schema json (`#8179 <https://github.com/autowarefoundation/autoware.universe/issues/8179>`_)
  * Refractored the parameters, build the schema file, updated the readme file.
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_multi_object_tracker): fix functionConst (`#8290 <https://github.com/autowarefoundation/autoware.universe/issues/8290>`_)
  * fix:functionConst
  * fix:functionConst
  * fix:clang format
  ---------
* fix(autoware_multi_object_tracker): revert latency reduction logic and bring back to timer trigger (`#8277 <https://github.com/autowarefoundation/autoware.universe/issues/8277>`_)
  * fix: revert latency reduction logic and bring back to timer trigger
  * style(pre-commit): autofix
  * chore: remove unused variables
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_multi_object_tracker): fix uninitMemberVar (`#8335 <https://github.com/autowarefoundation/autoware.universe/issues/8335>`_)
  fix:uninitMemberVar
* fix(autoware_multi_object_tracker): fix passedByValue (`#8231 <https://github.com/autowarefoundation/autoware.universe/issues/8231>`_)
  fix:passedByValue
* fix(multi_object_tracker, object_merger, radar_object_tracker, tracking_object_merger): fix knownConditionTrueFalse warnings (`#8137 <https://github.com/autowarefoundation/autoware.universe/issues/8137>`_)
  * fix: cppcheck knownConditionTrueFalse
  * fix
  * fix
  ---------
* fix(autoware_multi_object_tracker): missing parameter schema path fix (`#8120 <https://github.com/autowarefoundation/autoware.universe/issues/8120>`_)
  fix: missing parameter schema path fix
* fix(multi_object_tracker): fix funcArgNamesDifferent (`#8079 <https://github.com/autowarefoundation/autoware.universe/issues/8079>`_)
  fix:funcArgNamesDifferent
* refactor(multi_object_tracker): bring parameter schema to new package folder (`#8105 <https://github.com/autowarefoundation/autoware.universe/issues/8105>`_)
  refactor: bring parameter schema to new package folder
* refactor(multi_object_tracker)!: add package name prefix of autoware\_ (`#8083 <https://github.com/autowarefoundation/autoware.universe/issues/8083>`_)
  * refactor: rename multi_object_tracker package to autoware_multi_object_tracker
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Boyang, Esteve Fernandez, Ryuta Kambe, Taekjin LEE, Yutaka Kondo, kminoda, kobayu858

0.26.0 (2024-04-03)
-------------------
