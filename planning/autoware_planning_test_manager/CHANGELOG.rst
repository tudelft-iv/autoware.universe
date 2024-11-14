^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_test_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(autoware_test_utils): sanitizer header (`#9174 <https://github.com/autowarefoundation/autoware.universe/issues/9174>`_)
* refactor(component_interface_specs): prefix package and namespace with autoware (`#9094 <https://github.com/autowarefoundation/autoware.universe/issues/9094>`_)
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware.universe/issues/7640>`_)
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware.universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware.universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware.universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(route_handler)!: rename to include/autoware/{package_name}  (`#7530 <https://github.com/autowarefoundation/autoware.universe/issues/7530>`_)
  refactor(route_handler)!: rename to include/autoware/{package_name}
* refactor(test_utils): move to common folder (`#7158 <https://github.com/autowarefoundation/autoware.universe/issues/7158>`_)
  * Move autoware planning test manager to autoware namespace
  * fix package share directory for behavior path planner
  * renaming files and directory
  * rename variables that has planning_test_utils in its name.
  * use autoware namespace for test utils
  * move folder to common
  * update .pages file
  * fix test error
  * removed obstacle velocity limiter test artifact
  * remove namespace from planning validator, it has using keyword
  ---------
* refactor(route_handler): route handler add autoware prefix (`#7341 <https://github.com/autowarefoundation/autoware.universe/issues/7341>`_)
  * rename route handler package
  * update packages dependencies
  * update include guards
  * update includes
  * put in autoware namespace
  * fix formats
  * keep header and source file name as before
  ---------
* refactor(planning_validator)!: prefix package and namespace with autoware (`#7320 <https://github.com/autowarefoundation/autoware.universe/issues/7320>`_)
  * add autoware\_ prefix to planning_validator
  * add prefix to package name in .pages
  * fix link of the image
  ---------
* feat!: replace autoware_auto_msgs with autoware_msgs for planning modules (`#7246 <https://github.com/autowarefoundation/autoware.universe/issues/7246>`_)
  Co-authored-by: Cynthia Liu <cynthia.liu@autocore.ai>
  Co-authored-by: NorahXiong <norah.xiong@autocore.ai>
  Co-authored-by: beginningfan <beginning.fan@autocore.ai>
* chore(autoware_velocity_smoother, autoware_path_optimizer): rename packages (`#7202 <https://github.com/autowarefoundation/autoware.universe/issues/7202>`_)
  * chore(autoware_path_optimizer): rename package and namespace
  * chore(autoware_static_centerline_generator): rename package and namespace
  * chore: update module name
  * chore(autoware_velocity_smoother): rename package and namespace
  * chore(tier4_planning_launch): update module name
  * chore: update module name
  * fix: test
  * fix: test
  * fix: test
  ---------
* docs(planning_test_utils): update purpose of the package and add lanelet map images (`#7077 <https://github.com/autowarefoundation/autoware.universe/issues/7077>`_)
  * docs(planning_test_utils): Add explanation
  * remove autoware prefix from autoware planning test manager
  * fix document
  * remove implemented test part
  ---------
* refactor(planning_test_utils): remove route_handler dependencies (`#7005 <https://github.com/autowarefoundation/autoware.universe/issues/7005>`_)
  * refactor(planning_test_utils): remove route_handler dependencies
  * style(pre-commit): autofix
  * Fix precommit
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(autoware_planning_test_manager): rename package (`#6995 <https://github.com/autowarefoundation/autoware.universe/issues/6995>`_)
  * refactor(autoware_planning_test_manager): rename package
  * rename file
  * Add maintainer for planning test utils
  * Add route handler back into package.xml
  ---------
* Contributors: Esteve Fernandez, Kosuke Takeuchi, Kyoichi Sugahara, Mamoru Sobue, Ryohsuke Mitsudome, Satoshi OTA, Takayuki Murooka, Yutaka Kondo, Zulfaqar Azmi, mkquda

0.26.0 (2024-04-03)
-------------------
