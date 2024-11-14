^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_detected_object_validation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(object_recognition_utils): add autoware prefix to object_recognition_utils (`#8946 <https://github.com/autowarefoundation/autoware.universe/issues/8946>`_)
* refactor(detected_object_validation): rework parameters (`#7750 <https://github.com/autowarefoundation/autoware.universe/issues/7750>`_)
  * refactor(detected_object_validation): rework parameters
  * style(pre-commit): autofix
  * Update perception/detected_object_validation/schema/object_lanelet_filter.schema.json
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  * Update perception/detected_object_validation/schema/object_position_filter.schema.json
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  * refactor(detected_object_validation): rework parameters
  * refactor(detected_object_validation): rework parameters
  * refactor(detected_object_validation): rework parameters
  * style(pre-commit): autofix
  * refactor(detected_object_validation): rework parameters
  * refactor(detected_object_validation): rework parameters
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_detected_object_validation): fix functionStatic (`#8482 <https://github.com/autowarefoundation/autoware.universe/issues/8482>`_)
  fix:functionStatic
* fix(autoware_detected_object_validation): fix cppcheck warnings of functionStatic (`#8256 <https://github.com/autowarefoundation/autoware.universe/issues/8256>`_)
  fix: deal with functionStatic warnings
* fix(autoware_detected_object_validation): fix functionConst (`#8285 <https://github.com/autowarefoundation/autoware.universe/issues/8285>`_)
  fix: functionConst
* perf(autoware_detected_object_validation): reduce lanelet_filter processing time  (`#8240 <https://github.com/autowarefoundation/autoware.universe/issues/8240>`_)
  * add local r-tree for fast searching
  change to _func\_\_
  add more debug
  use local rtree
  fix
  tmp update
  fix bug
  clean unused
  clean up
  * clean up
  * style(pre-commit): autofix
  * chore: Optimize object filtering and improve performance
  The code changes in `lanelet_filter.cpp` optimize the object filtering process by using the `empty()` function instead of checking the size of the `transformed_objects.objects` vector. This change improves performance and simplifies the code logic.
  Refactor the code to use `empty()` instead of `size()` for checking if the `transformed_objects.objects` vector is empty. This change improves readability and performance.
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* refactor(probabilistic_occupancy_grid_map, occupancy_grid_map_outlier_filter): add autoware\_ prefix to package name (`#8183 <https://github.com/autowarefoundation/autoware.universe/issues/8183>`_)
  * chore: fix package name probabilistic occupancy grid map
  * fix: solve launch error
  * chore: update occupancy_grid_map_outlier_filter
  * style(pre-commit): autofix
  * refactor: update package name to autoware_probabilistic_occupancy_grid_map on a test
  * refactor: rename folder of occupancy_grid_map_outlier_filter
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* refactor(detected_object_validation)!: add package name prefix of autoware\_ (`#8122 <https://github.com/autowarefoundation/autoware.universe/issues/8122>`_)
  refactor: rename detected_object_validation to autoware_detected_object_validation
* Contributors: Batuhan Beytekin, Esteve Fernandez, Hayate TOBA, Masaki Baba, Taekjin LEE, Yoshi Ri, Yutaka Kondo, kobayu858, taisa1

0.26.0 (2024-04-03)
-------------------
