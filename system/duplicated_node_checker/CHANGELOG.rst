^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package duplicated_node_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix(duplicated_node_checker): fix unusedFunction (`#8579 <https://github.com/autowarefoundation/autoware.universe/issues/8579>`_)
  fix: unusedFunction
  Co-authored-by: kobayu858 <129580202+kobayu858@users.noreply.github.com>
* feat(duplicated_node_checker): add duplicate nodes to ignore (`#7959 <https://github.com/autowarefoundation/autoware.universe/issues/7959>`_)
  * feat: add duplicate nodes to ignore
  * remove talker
  * newline
  * commments
  * pre-commit and sign
  * rviz->rviz2
  ---------
  Co-authored-by: Dmitrii Koldaev <dmitrii.koldaev@tier4.jp>
* Contributors: Dmitrii Koldaev, Hayate TOBA, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
* chore(duplicate_node_checker): print duplication name (`#6488 <https://github.com/autowarefoundation/autoware.universe/issues/6488>`_)
* feat(duplicated_node_checker): add duplicated node names to msg (`#5382 <https://github.com/autowarefoundation/autoware.universe/issues/5382>`_)
  * add duplicated node names to msg
  * align with launcher repository
  ---------
* feat(duplicated_node_checker): add packages to check duplication of node names in ros2 (`#5286 <https://github.com/autowarefoundation/autoware.universe/issues/5286>`_)
  * add implementation for duplicated node checking
  * update the default parameters of system_error_monitor to include results from duplication check
  * style(pre-commit): autofix
  * fix typo in readme
  * update license
  * change module to the system module
  * follow json schema: 1. update code to start without default 2. add schema/config/readme/launch accordingly
  * add duplicated node checker to launch
  * style(pre-commit): autofix
  * fix var name to config for uniform launch
  * Update system/duplicated_node_checker/README.md
  * Update system/duplicated_node_checker/README.md
  ---------
  Co-authored-by: Owen-Liuyuxuan <uken.ryu@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
* Contributors: Kyoichi Sugahara, Mamoru Sobue, Yuxuan Liu
