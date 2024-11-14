^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_probabilistic_occupancy_grid_map
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* feat(probabilistic_occupancy_grid_map): add time_keeper (`#8601 <https://github.com/autowarefoundation/autoware.universe/issues/8601>`_)
  * add time_keeper
  * add option for time keeper
  * correct namespace
  * set default to false
  * add scope and timekeeper
  * remove scope and add comment for scopes
  * mod comment
  * change comment
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * fix variable shadowing
  ---------
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
* fix(autoware_probabilistic_occupancy_grid_map): fix unusedFunction (`#8574 <https://github.com/autowarefoundation/autoware.universe/issues/8574>`_)
  fix:unusedFunction
* fix(autoware_probabilistic_occupancy_grid_map): fix functionConst (`#8426 <https://github.com/autowarefoundation/autoware.universe/issues/8426>`_)
  fix:functionConst
* fix(autoware_probabilistic_occupancy_grid_map): fix uninitMemberVar (`#8333 <https://github.com/autowarefoundation/autoware.universe/issues/8333>`_)
  fix:uninitMemberVar
* fix(autoware_probabilistic_occupancy_grid_map): fix functionConst (`#8289 <https://github.com/autowarefoundation/autoware.universe/issues/8289>`_)
  fix:functionConst
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
* Contributors: Masaki Baba, Yoshi Ri, Yutaka Kondo, kobayu858

0.26.0 (2024-04-03)
-------------------
