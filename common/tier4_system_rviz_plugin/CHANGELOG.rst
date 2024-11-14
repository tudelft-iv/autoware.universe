^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tier4_system_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* feat(system_error_monitor): remove system error monitor (`#8929 <https://github.com/autowarefoundation/autoware.universe/issues/8929>`_)
  * feat: delete-system-error-monitor-from-autoware
  * feat: remove unnecessary params
  ---------
* fix(tier4_system_rviz_plugin): fix cppcheck warning of virtualCallInConstructor (`#8378 <https://github.com/autowarefoundation/autoware.universe/issues/8378>`_)
  fix: deal with virtualCallInConstructor warning
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware.universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat!: replace autoware_auto_msgs with autoware_msgs for common modules (`#7239 <https://github.com/autowarefoundation/autoware.universe/issues/7239>`_)
  Co-authored-by: Cynthia Liu <cynthia.liu@autocore.ai>
  Co-authored-by: NorahXiong <norah.xiong@autocore.ai>
  Co-authored-by: beginningfan <beginning.fan@autocore.ai>
* Contributors: Ryohsuke Mitsudome, Takayuki Murooka, TetsuKawa, Yutaka Kondo, taisa1

0.26.0 (2024-04-03)
-------------------
* feat(tier4_system_rviz_plugin): improve visualization results and logics (`#5222 <https://github.com/autowarefoundation/autoware.universe/issues/5222>`_)
  * Add Init Localization and Init Planning Check; Add error list check
  * style(pre-commit): autofix
  * int casting problem updated
  * style(pre-commit): autofix
  * improve if statement writing styles
  * style(pre-commit): autofix
  * FIX ci
  * subscribe to ADAPI for more stable status checking
  * style(pre-commit): autofix
  * try using level attribute only
  * fix hpp so that it just look like the original hpp
  * style(pre-commit): autofix
  * set default value to NO_FAULT
  ---------
  Co-authored-by: Owen-Liuyuxuan <uken.ryu@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(common): extern template for motion_utils / remove tier4_autoware_utils.hpp / remove motion_utis.hpp (`#5027 <https://github.com/autowarefoundation/autoware.universe/issues/5027>`_)
* feat(tier4_system_rviz_plugin): add package (`#4945 <https://github.com/autowarefoundation/autoware.universe/issues/4945>`_)
  * feat(tier4_system_rviz_plugin): add package
  * update maintainer
  * style(pre-commit): autofix
  * minor update
  * style(pre-commit): autofix
  * update some arguments
  * style(pre-commit): autofix
  * fix pre-commit
  * style(pre-commit): autofix
  * update maintainer
  * fix test
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Mamoru Sobue, Yuxuan Liu, kminoda
