^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_traffic_light_fine_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(tensorrt_common)!: fix namespace, directory structure & move to perception namespace (`#9099 <https://github.com/autowarefoundation/autoware.universe/issues/9099>`_)
  * refactor(tensorrt_common)!: fix namespace, directory structure & move to perception namespace
  * refactor(tensorrt_common): directory structure
  * style(pre-commit): autofix
  * fix(tensorrt_common): correct package name for logging
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* feat(autoware_tensorrt_yolox): add GPU - CUDA device option (`#8245 <https://github.com/autowarefoundation/autoware.universe/issues/8245>`_)
  * init CUDA device option
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_traffic_light_fine_detector): fix unusedFunction (`#8583 <https://github.com/autowarefoundation/autoware.universe/issues/8583>`_)
  fix:unusedFunction
* fix(autoware_traffic_light_fine_detector): fix passedByValue (`#8237 <https://github.com/autowarefoundation/autoware.universe/issues/8237>`_)
  fix:passedByValue
* fix(traffic_light_fine_detector): fix funcArgNamesDifferent (`#8154 <https://github.com/autowarefoundation/autoware.universe/issues/8154>`_)
  fix:funcArgNamesDifferent
* fix(autoware_traffic_light_fine_detector): fix constParameterReference (`#8146 <https://github.com/autowarefoundation/autoware.universe/issues/8146>`_)
  * fix:constParameterReference
  * fix:constParameterReference
  * fix:constParameterReference
  ---------
* refactor(tensorrt_yolox)!: fix namespace and directory structure (`#7992 <https://github.com/autowarefoundation/autoware.universe/issues/7992>`_)
  * refactor: add autoware namespace prefix to `tensorrt_yolox`
  * refactor: apply `autoware` namespace to tensorrt_yolox
  * chore: update CODEOWNERS
  * fix: resolve `yolox_tiny` to work
  ---------
* refactor(traffic_light\_*)!: add package name prefix of autoware\_ (`#8159 <https://github.com/autowarefoundation/autoware.universe/issues/8159>`_)
  * chore: rename traffic_light_fine_detector to autoware_traffic_light_fine_detector
  * chore: rename traffic_light_multi_camera_fusion to autoware_traffic_light_multi_camera_fusion
  * chore: rename traffic_light_occlusion_predictor to autoware_traffic_light_occlusion_predictor
  * chore: rename traffic_light_classifier to autoware_traffic_light_classifier
  * chore: rename traffic_light_map_based_detector to autoware_traffic_light_map_based_detector
  * chore: rename traffic_light_visualization to autoware_traffic_light_visualization
  ---------
* Contributors: Amadeusz Szymko, Ismet Atabay, Kotaro Uetake, Taekjin LEE, Yutaka Kondo, kobayu858

0.26.0 (2024-04-03)
-------------------
