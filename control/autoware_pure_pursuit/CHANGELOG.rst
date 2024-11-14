^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_pure_pursuit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix(pure_pursuit): add autoware\_ prefix in launch file (`#8687 <https://github.com/autowarefoundation/autoware.universe/issues/8687>`_)
* fix(autoware_pure_pursuit): fix unusedFunction (`#8552 <https://github.com/autowarefoundation/autoware.universe/issues/8552>`_)
  fix:unusedFunction
* fix(autoware_pure_pursuit): fix redundantInitialization redundantInitialization (`#8225 <https://github.com/autowarefoundation/autoware.universe/issues/8225>`_)
  * fix(autoware_pure_pursuit): fix redundantInitialization redundantInitialization
  * style(pre-commit): autofix
  * Update control/autoware_pure_pursuit/src/autoware_pure_pursuit_core/interpolate.cpp
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
* fix(autoware_pure_pursuit): fix shadowVariable (`#7932 <https://github.com/autowarefoundation/autoware.universe/issues/7932>`_)
  * fix:shadowVariable
  * refactor: using range-based for loop
  ---------
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware.universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware.universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware.universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(control)!: refactor directory structures of the trajectory followers (`#7521 <https://github.com/autowarefoundation/autoware.universe/issues/7521>`_)
  * control_traj
  * add follower_node
  * fix
  ---------
* refactor(pure_pursuit): prefix package and namespace with autoware\_ (`#7301 <https://github.com/autowarefoundation/autoware.universe/issues/7301>`_)
  * RT1-6683 add autoware prefix to package and namepace
  * fix precommit
  ---------
* Contributors: Kosuke Takeuchi, Ryohsuke Mitsudome, Ryuta Kambe, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo, Zulfaqar Azmi, kobayu858

0.26.0 (2024-04-03)
-------------------
