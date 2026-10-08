# DCW RGB-D 카메라 Gazebo 시뮬레이션

DCW는 `arm4`에 부착된 RGB-D 카메라로 구성한다. Gazebo는 컬러 영상과 깊이 영상을 생성하고, ROS 2에서는 `ros_gz_bridge`를 통해 이를 사용할 수 있다.

## 좌표계와 TF

```text
arm4
└── DCW2                         # arm4에 고정된 URDF 링크
    └── dcw_rgbd_frame           # Gazebo RGB-D 점군 프레임 (+X 전방)
        └── dcw_depth_optical_frame  # ROS optical frame (+Z 전방)
```

`robot_state_publisher`는 URDF의 fixed joint를 바탕으로 `arm4 -> DCW2` TF를 발행한다. 카메라가 팔을 따라 움직이도록 Gazebo 센서는 `arm4` 링크 내부에 두고, `DCW2` 프레임을 기준으로 pose를 지정한다. Gazebo RGB-D 점군은 `+X`가 전방인 `dcw_rgbd_frame`을 header frame으로 사용한다.

> 현재 SDF에서는 `DCW2`의 mesh가 `arm4` 링크에 합쳐져 있다. 따라서 센서는 별도 링크를 만들지 않아도 `arm4` 안에 추가할 수 있다.

## 1. Gazebo Sensors 시스템 활성화

`src/robot_description/worlds/n1_imu.world`의 `<world>` 안에 다음 플러그인을 추가한다.

```xml
<plugin filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

이 플러그인이 렌더링 기반 카메라 센서를 실행한다.

## 2. RGB-D 센서 추가

`src/robot_description/gazebo/N1-Robot.sdf`의 `<link name='arm4'>` 내부에 다음 센서를 추가한다.

```xml
<sensor name="dcw_rgbd_camera" type="rgbd_camera">
  <!-- 렌즈 중심의 실제 장착 pose에 맞춰 조정한다. -->
  <pose relative_to="DCW2">0 0 0 0 0 0</pose>

  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>

  <!-- image, depth_image, camera_info의 공통 prefix -->
  <topic>/dcw/rgbd</topic>

  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.10</near>
      <far>5.0</far>
    </clip>
  </camera>
</sensor>
```

Gazebo는 다음 토픽을 발행한다.

```text
/dcw/rgbd/image        # 컬러 이미지
/dcw/rgbd/depth_image  # 픽셀별 깊이 이미지
/dcw/rgbd/camera_info  # 해상도와 내부 파라미터
/dcw/rgbd/points       # ROS optical frame 기준 RGB-D 점군
/dcw/rgbd/points_raw   # Gazebo +X 전방 좌표의 원본 점군
```

`<pose>`의 회전값은 DCW mesh의 렌즈 전방이 향하는 방향에 맞춰 조정해야 한다. Gazebo 카메라는 기본적으로 센서의 `+X` 방향을 바라본다. 현재 DCW2 프레임의 `+X`가 로봇 전방과 일치하므로 yaw 보정은 사용하지 않는다.

## 3. ROS 2 브리지 추가

`src/robot_description/launch/sim.launch.py`의 `ros_gz_bridge` `arguments` 목록에 다음을 추가한다.

```python
'/dcw/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image',
'/dcw/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
'/dcw/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
'/dcw/rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
```

ROS 2에서는 같은 이름으로 다음 타입의 토픽을 받는다.

```text
/dcw/rgbd/image        sensor_msgs/msg/Image
/dcw/rgbd/depth_image  sensor_msgs/msg/Image
/dcw/rgbd/camera_info  sensor_msgs/msg/CameraInfo
/dcw/rgbd/points       sensor_msgs/msg/PointCloud2

`gz_pointcloud_to_optical.py`는 브리지된 원본 점군을 `/dcw/rgbd/points_raw`로 받고, Gazebo 좌표 `(x, y, z)`를 ROS optical 좌표 `(-y, -z, x)`로 변환해 `/dcw/rgbd/points`로 발행한다.
```

## 4. Point cloud frame과 optical frame

Gazebo RGB-D 점군은 센서의 `+X`가 전방인 좌표를 발행하므로, 메시지 `header.frame_id`는 센서 pose와 같은 `dcw_rgbd_frame`으로 설정한다. 그래야 RViz가 점군을 올바른 방향에 배치한다.

ROS camera optical frame은 `+Z`가 전방, `+X`가 오른쪽, `+Y`가 아래 방향이다. 영상 알고리즘에 필요할 경우 `dcw_rgbd_frame -> dcw_depth_optical_frame` fixed joint를 별도로 유지한다.

일반적인 Gazebo camera frame에서 optical frame으로의 회전은 다음과 같다.

```xml
<origin xyz="0 0 0" rpy="-1.5708 0 -1.5708" />
```

센서 메시지의 `header.frame_id`가 이 TF 트리의 프레임과 일치하는지 확인한다. RGB-D 점군은 `dcw_rgbd_frame`을 사용해야 하며, 일치하지 않으면 점군을 `arm4`, `DCW2`, `odom` 기준으로 올바르게 변환할 수 없다.

## 5. 검증

시뮬레이터를 실행한 뒤 토픽과 메시지를 확인한다.

```bash
gz topic -l | rg 'dcw|rgbd'
ros2 topic list | rg 'dcw|rgbd'
ros2 topic echo /dcw/rgbd/camera_info --once
ros2 run tf2_ros tf2_echo arm4 DCW2
```

Gazebo 버전에 따라 실제 생성 토픽을 먼저 `gz topic -l`로 확인하고, 그 이름을 `parameter_bridge`에 사용한다.
