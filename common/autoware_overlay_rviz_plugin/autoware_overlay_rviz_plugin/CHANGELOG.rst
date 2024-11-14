^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_overlay_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix(speed_display): always show speed positive and depend on gear for negatives (`#8957 <https://github.com/autowarefoundation/autoware.universe/issues/8957>`_)
* style: update state panel plugin (`#8846 <https://github.com/autowarefoundation/autoware.universe/issues/8846>`_)
* feat: add color customization to gear speed mission speed limit and traffic displays (`#8142 <https://github.com/autowarefoundation/autoware.universe/issues/8142>`_)
  feat: Add color customization to gear, speed, mission,speedlimit and traffic displays
* fix(autoware_overlay_rviz_plugin): topic type of traffic light (`#8098 <https://github.com/autowarefoundation/autoware.universe/issues/8098>`_)
  * fix(autoware_overlay_rviz_plugin): topic type of traffic light
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat: add handle angle scale property to signal display (`#7774 <https://github.com/autowarefoundation/autoware.universe/issues/7774>`_)
  * feat: add handle angle scale property to signal display
  * fix: set default steering angle to 0.0° when not received
  * fix: set default steering angle to N/A when not received and check for msg_ptr instead of float value
  * chore: update steering wheel font size and wheel icon center is bigger
  * chore: update steering wheel display to center the steering angle text
  * chore: align steering angle text in both negative and positive cases
  ---------
* feat!: replace autoware_auto_msgs with autoware_msgs for common modules (`#7239 <https://github.com/autowarefoundation/autoware.universe/issues/7239>`_)
  Co-authored-by: Cynthia Liu <cynthia.liu@autocore.ai>
  Co-authored-by: NorahXiong <norah.xiong@autocore.ai>
  Co-authored-by: beginningfan <beginning.fan@autocore.ai>
* fix(autoware_overlay_rviz_plugin): fix subs and cleanup (`#6978 <https://github.com/autowarefoundation/autoware.universe/issues/6978>`_)
* feat: update rviz2 overlay (`#6883 <https://github.com/autowarefoundation/autoware.universe/issues/6883>`_)
* feat(autoware_overlay_rviz_plugin): get the current traffic light (`#6899 <https://github.com/autowarefoundation/autoware.universe/issues/6899>`_)
* Contributors: Khalil Selyan, M. Fatih Cırıt, Maxime CLEMENT, Ryohsuke Mitsudome, Tomohito ANDO, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
* build(autoware_overlay_rviz_plugin): add missing ament_cmake_auto dependency (`#6519 <https://github.com/autowarefoundation/autoware.universe/issues/6519>`_)
* feat: update vehicle overlay plugin (`#6323 <https://github.com/autowarefoundation/autoware.universe/issues/6323>`_)
* Contributors: Esteve Fernandez, Khalil Selyan
