^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package automatic_pose_initializer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix(automatic_pose_initializer): fix plugin name (`#7035 <https://github.com/autowarefoundation/autoware.universe/issues/7035>`_)
  Fixed automatic_pose_initializer plugin name
* feat(automatic_pose_initializer): componentize node (`#7021 <https://github.com/autowarefoundation/autoware.universe/issues/7021>`_)
* Contributors: SakodaShintaro, Takagi, Isamu, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
* chore: update api package maintainers (`#6086 <https://github.com/autowarefoundation/autoware.universe/issues/6086>`_)
  * update api maintainers
  * fix
  ---------
* build: mark autoware_cmake as <buildtool_depend> (`#3616 <https://github.com/autowarefoundation/autoware.universe/issues/3616>`_)
  * build: mark autoware_cmake as <buildtool_depend>
  with <build_depend>, autoware_cmake is automatically exported with ament_target_dependencies() (unecessary)
  * style(pre-commit): autofix
  * chore: fix pre-commit errors
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
* chore(default_ad_api): add yukkysaito and mitsudome-r to maintainer (`#3440 <https://github.com/autowarefoundation/autoware.universe/issues/3440>`_)
  * chore(default_ad_api): add yukkysaito to maintainer
  * add mitsudome-r instead of kenji-miyake
  ---------
* chore: add api maintainers (`#2361 <https://github.com/autowarefoundation/autoware.universe/issues/2361>`_)
* feat(autoware_ad_api_msgs): replace adapi message (`#1897 <https://github.com/autowarefoundation/autoware.universe/issues/1897>`_)
* fix(automatic_pose_initializer): fix starvation (`#1756 <https://github.com/autowarefoundation/autoware.universe/issues/1756>`_)
* feat(default_ad_api): add localization api  (`#1431 <https://github.com/autowarefoundation/autoware.universe/issues/1431>`_)
  * feat(default_ad_api): add localization api
  * docs: add readme
  * feat: add auto initial pose
  * feat(autoware_ad_api_msgs): define localization interface
  * fix(default_ad_api): fix interface definition
  * feat(default_ad_api): modify interface version api to use spec package
  * feat(default_ad_api): modify interface version api to use spec package
  * fix: pre-commit
  * fix: pre-commit
  * fix: pre-commit
  * fix: copyright
  * feat: split helper package
  * fix: change topic name to local
  * fix: style
  * fix: style
  * fix: style
  * fix: remove needless keyword
  * feat: change api helper node namespace
  * fix: fix launch file path
* Contributors: Kosuke Takeuchi, Takagi, Isamu, Vincent Richard
