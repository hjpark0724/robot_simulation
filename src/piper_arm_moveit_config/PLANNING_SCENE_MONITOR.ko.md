# Planning Scene Monitor (PSM)

이 문서는 [MoveIt 공식 Planning Scene Monitor 문서](https://moveit.picknik.ai/main/doc/concepts/planning_scene_monitor.html)의 주제와 용어를 따라 핵심 내용을 한국어로 정리하고, 마지막에 듀얼 Piper Arm 적용 내용을 덧붙인 요약입니다.

계획 장면(Planning Scene)은 로봇의 상태와 주변 환경을 함께 표현합니다. Planning Scene Monitor는 관절 상태와 환경 정보를 받아 장면을 갱신하고, 여러 구성요소가 이를 읽고 변경할 수 있게 관리합니다. `move_group`과 RViz의 계획 장면 플러그인은 각각 PSM을 둡니다. `move_group` 쪽 PSM은 `/planning_scene`을 받고, 갱신된 장면을 `/monitored_planning_scene`으로 발행해 RViz에 전달합니다.

![MoveIt Planning Scene Monitor의 구성과 데이터 흐름](https://moveit.picknik.ai/main/_images/planning_scene_monitor.svg)

그림 출처: [MoveIt 공식 문서의 원본 SVG](https://moveit.picknik.ai/main/_images/planning_scene_monitor.svg).

## 주변 환경 모니터 (World Geometry Monitor)

로봇의 LiDAR·깊이 카메라 등에서 얻은 관측과 사용자가 제공한 물체 정보를 합쳐 주변 환경을 표현합니다. 센서 관측은 점유 지도 모니터에서 3D 공간 정보로 만들고, 별도의 물체 정보는 계획 장면 갱신을 통해 반영할 수 있습니다.

## 3D 인식 (3D Perception)

점유 지도 모니터는 입력 종류에 맞는 플러그인을 사용합니다.

| 센서 입력 | MoveIt 갱신기 |
| --- | --- |
| 점군 | `PointCloudOccupancyMapUpdater` |
| 깊이 영상 | `DepthImageOccupancyMapUpdater` |

필요하면 다른 입력용 갱신기도 플러그인으로 추가할 수 있습니다.

## OctoMap

점유 지도 모니터는 관측한 공간을 OctoMap에 저장합니다. MoveIt은 이 지도를 충돌 검사 라이브러리인 FCL에 전달할 수 있습니다. OctoMap은 각 공간 셀의 점유 확률을 표현할 수 있지만, 공식 문서에 따르면 MoveIt은 그 확률값 자체를 사용하지 않습니다.

### 깊이 영상 점유 지도 갱신기

`DepthImageOccupancyMapUpdater`에는 로봇 자신을 걸러내는 기능이 있습니다. 현재 로봇 상태를 참고해 깊이 영상에 보이는 로봇 부분을 점유 지도에서 제외합니다.

## 이 패키지에 적용하기

`demo.launch.py`에서 `/joint_states`는 `joint_state_broadcaster`가 발행하며, `robot_state_publisher`가 TF를 제공합니다. 현재 패키지에는 카메라 입력이나 3D 점유 지도 갱신기 설정이 없습니다. 인식을 흉내 내려면 가상의 물체 위치·크기를 생성하는 노드가 충돌 물체 또는 계획 장면 갱신 메시지를 보내도록 구성할 수 있습니다. 이때 물체의 기준 좌표계가 로봇 TF와 맞아야 합니다.

## 확인할 토픽

| 토픽 | 확인할 내용 |
| --- | --- |
| `/joint_states` | 계획의 시작 상태에 사용할 현재 관절 값 |
| `/tf`, `/tf_static` | 로봇과 물체 좌표를 연결할 프레임 변환 |
| `/planning_scene`, `/collision_object` | 외부 노드가 제공하는 장면·충돌 물체 갱신 |
| `/monitored_planning_scene` | `move_group` 쪽에서 반영한 계획 장면 |

실행 중인 연결은 `ros2 topic info -v /joint_states`와 `ros2 topic info -v /monitored_planning_scene`로 확인할 수 있습니다. RViz를 종료하면 RViz 쪽 구독 연결은 사라집니다.
