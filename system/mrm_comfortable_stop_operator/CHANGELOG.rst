^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrm_comfortable_stop_operator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* chore(mrm_comfortable_stop_operator): remove unused main file (`#7191 <https://github.com/autowarefoundation/autoware.universe/issues/7191>`_)
* Contributors: Takagi, Isamu, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
* chore(mrm_emergency_stop_operator): add a maintainer for mrm operator… (`#3489 <https://github.com/autowarefoundation/autoware.universe/issues/3489>`_)
  chore(mrm_emergency_stop_operator): add a maintainer for mrm operator packages
* style: fix typos (`#3617 <https://github.com/autowarefoundation/autoware.universe/issues/3617>`_)
  * style: fix typos in documents
  * style: fix typos in package.xml
  * style: fix typos in launch files
  * style: fix typos in comments
  ---------
* build: mark autoware_cmake as <buildtool_depend> (`#3616 <https://github.com/autowarefoundation/autoware.universe/issues/3616>`_)
  * build: mark autoware_cmake as <buildtool_depend>
  with <build_depend>, autoware_cmake is automatically exported with ament_target_dependencies() (unecessary)
  * style(pre-commit): autofix
  * chore: fix pre-commit errors
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
* chore: sync files (`#3227 <https://github.com/autowarefoundation/autoware.universe/issues/3227>`_)
  * chore: sync files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(mrm_emergency_stop_operator): fix parameter loading in mrm operators (`#2378 <https://github.com/autowarefoundation/autoware.universe/issues/2378>`_)
  * fix(mrm_emergency_stop_operator): fix parameter loading in mrm operators
  * ci(pre-commit): autofix
  * fix(mrm_emergency_stop_operator): remove os import
  * fix(mrm_emergency_stop_operator): remove unused packages
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(emergency_handler): add a selector for multiple MRM behaviors (`#2070 <https://github.com/autowarefoundation/autoware.universe/issues/2070>`_)
  * feat(emergency_handler): add mrm command and status publishers
  * feat(autoware_ad_api_msgs): define mrm operation srv and mrm status msg
  * feat(emergency_handler): add mrm clients and subscribers
  * feat(mrm_comfortable_stop_operator): ready ros2 node template
  * feat(mrm_comfortable_stop_operator): implemented
  * feat(mrm_comfortable_stop_operator): implement as component
  * chore(mrm_comfortable_stop_operator): add a launch script
  * refactor(mrm_comfortable_stop_operator): remove a xml launch file
  * feat(autoware_ad_api_msgs): change mrm status msg
  * feat(emergency_handler): add mrm operator and mrm behavior updater
  * feat(emergency_handler): add mrm behavior state machine
  * feat(emergency_handler): remap io names
  * fix(emergency_handler): fix request generation
  * fix(emergency_handler): add multi thread execution for service
  * feat(vehicle_cmd_gate): add mrm operation service and status publisher
  * refactor(mrm_comfortable_stop_operator): use MRMBehaviorStatus struct
  * fix(mrm_comfortable_stop_operator): add time stamp for status
  * feat(vehicle_cmd_gate): change system emergency state by mrm operation
  * chore(autoware_ad_api_msgs): remove rti_operating state from mrm status
  * feat(mrm_sudden_stop_operator): add mrm_sudden_stop_operator
  * refactor(autoware_ad_api_msgs): rename from mrm status to mrm state
  * fix(mrm_comfortable_stop_operator): set qos for velocity limit publisher
  * feat(emergency_handler): add mrm state publisher
  * feat(vehicle_cmd_gate): add subscription for mrm_state
  * fix(mrm_sudden_stop_operator): fix control command topic name
  * feat(vehicle_cmd_gate): pub emergency control_cmd according to mrm state
  * feat(emergency_handler): remove emergency control_cmd publisher
  * chore(tier4_control_launch): remap mrm state topic
  * feat(tier4_system_launch): launch mrm operators
  * fix(emergency_handler): fix autoware_ad_api_msgs to autoware_adapi_v1_msgs
  * fix(vehicle_cmd_gate): remove subscribers for emergency_state and mrm operation
  * fix(vehicle_cmd_gate): fix system emergency condition
  * fix(emergency_handler): add stamp for mrm_state
  * fix(mrm_emergency_stop_operator): rename sudden stop to emergency stop
  * fix(vehicle_cmd_gate): remove emergency stop status publisher
  * fix(emergency_handler): replace emergency state to mrm state
  * feat(mrm_emergency_stop_operator): add is_available logic
  * feat(emergency_handler): add use_comfortable_stop param
  * refactor(emergency_handler): rename getCurrentMRMBehavior
  * feat(emergency_handler): add mrm available status for ready conditions
  * feat(emergency_handler): add readme
  * fix(mrm_comfortable_stop_operator): fix update rate
  * refactor(emergency_handler): move MRMBehaviorStatus msg to tier4_system_msgs
  * feat(emergency_handler): describe new io for emergency_handler
  * fix(emergency_handler): remove extra settings
  * fix(mrm_emergency_stop_operator): fix is_available condition
  * fix(mrm_emergency_stop_operator): fix typo
  * ci(pre-commit): autofix
  * fix(mrm_emergency_stop_operator): remove extra descriptions on config files
  * fix(mrm_comfortable_stop_operator): fix typo
  * chore(mrm_comfortable_stop_operator): change words
  * chore(mrm_comfortable_stop_operator): change maintainer infomation
  * fix(emergency_handler): fix acronyms case
  * chore(emergency_handler): add a maintainer
  * fix(emergency_handler): fix to match msg changes
  * fix(vehicle_cmd_gate): remove an extra include
  * ci(pre-commit): autofix
  * fix(emergency_handler): fix topic name spaces
  * fix(emergency_handler): fix acronyms case
  * chore(tier4_system_launch): add a mrm comfortable stop parameter
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Kenji Miyake, Makoto Kurihara, Vincent Richard, awf-autoware-bot[bot]
