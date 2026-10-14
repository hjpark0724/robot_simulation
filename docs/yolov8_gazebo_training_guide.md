# Gazebo 영상으로 YOLOv8 물체 검출 모델 학습하기

이 문서는 이 저장소에서 **빨간 시험 상자**를 촬영하고 YOLOv8s를 학습해 ROS 2 물체 인식 노드에 적용한 과정을 재현한다. 뒤쪽의 [다른 물체로 바꾸기](#다른-물체로-바꾸기)를 따라 하면 새 물체에도 같은 절차를 적용할 수 있다. 명령은 저장소 루트(`robot_simulation`)에서 실행한다.

## 먼저 알아둘 것

| 용어 | 이 프로젝트에서의 의미 |
| --- | --- |
| RGB 이미지 | `/left_camera/rgbd/image`로 들어오는 카메라 사진. YOLO의 입력이다. |
| 바운딩 박스 | 사진 속 물체를 둘러싼 사각형. YOLO가 예측하는 값이다. |
| 클래스 | 검출할 물체 종류. 빨간 시험 상자는 `red_target_box` 하나로, ID는 `0`이다. |
| 라벨 | 이미지마다 정답 클래스와 바운딩 박스를 적은 `.txt` 파일. |
| `train` / `val` | 각각 학습용 / 학습에 사용하지 않는 검증용 이미지. |
| epoch | 모든 학습 이미지를 한 번씩 사용하는 학습 주기. |
| `best.pt` | 검증 결과가 가장 좋았던 PyTorch 가중치. 추가 학습에 사용한다. |
| `.onnx` | ROS C++ 노드에서 추론할 수 있도록 내보낸 모델. 학습된 가중치가 포함된다. |
| mAP50 | 검출 박스가 정답과 충분히 겹치는지 평가하는 지표. 값이 높아도 새로운 환경에서 잘 작동한다는 보장은 없다. |

전체 흐름은 **Gazebo 실행 → 이미지와 정답 박스 수집 → 학습 → 검증 → ONNX 변환 → ROS 노드 적용 → 실제 토픽 확인**이다. YOLO는 RGB 이미지에서 2D 박스를 찾는다. 이 저장소의 [인식 노드](../src/piper_arm_moveit_config/src/color_object_pose_node.cpp)는 같은 시각의 포인트클라우드에서 3D 표면점을 읽고, 지정한 물체 크기와 방향을 이용해 `/detected_object/pose`를 계산한다. YOLO 자체가 3D 중심이나 회전을 학습하는 것은 아니다.

## 1. 준비와 실행 환경

ROS 2 Jazzy, Gazebo Sim, `colcon`, 이 워크스페이스의 빌드가 필요하다. `color_object_pose_node` 빌드에는 `onnxruntime_vendor`가 필요하다. 설치된 ROS 환경에서는 다음처럼 준비한다.

```bash
cd /home/hjpark/CLionProjects/robot_simulation
sudo apt install ros-jazzy-onnxruntime-vendor
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select piper_arm_moveit_config --symlink-install
source install/setup.zsh
```

이 컴퓨터에서는 `onnxruntime_vendor`를 사용자 홈 아래에 설치해 빌드했다. 시스템 패키지가 없다면 아래 명령에서 **실제로 존재하는 vendor 경로**로 바꾼다. 위의 `apt` 방식이 가능한 환경이라면 이 별도 경로 설정은 필요 없다.

```bash
CMAKE_PREFIX_PATH=/home/hjpark/.local/ros-jazzy-onnxruntime/opt/ros/jazzy:$CMAKE_PREFIX_PATH \
  colcon build --packages-select piper_arm_moveit_config --symlink-install \
  --cmake-args -Donnxruntime_vendor_DIR=/home/hjpark/.local/ros-jazzy-onnxruntime/opt/ros/jazzy/share/onnxruntime_vendor/cmake
```

데이터 수집 스크립트는 ROS Python(`rclpy`), OpenCV(`cv2`), NumPy를 사용한다. 학습은 PyTorch, Ultralytics, ONNX 도구를 사용한다. ROS Python과 충돌을 피하려면 **수집은 ROS가 설치된 시스템 Python으로**, **학습은 별도 가상 환경으로** 실행한다. 새 학습 환경의 한 예는 다음과 같다. PyTorch 설치는 플랫폼에 따라 달라질 수 있으므로 먼저 가져오기가 되는지 확인한다.

```bash
python3 -m venv /tmp/piper_yolo_venv
/tmp/piper_yolo_venv/bin/python -m pip install --upgrade pip
/tmp/piper_yolo_venv/bin/python -m pip install torch torchvision ultralytics onnx onnxslim polars
/tmp/piper_yolo_venv/bin/python -c 'import torch, ultralytics, onnx; print(torch.__version__, ultralytics.__version__)'
python3 -c 'import rclpy, cv2, numpy; print("collector dependencies OK")'
```

이번 학습에서는 Ultralytics `8.4.177`, CPU의 PyTorch `2.14.0+cpu`, ONNX `1.23.0`을 사용했다. 당시 `/tmp/robot_yolo_training_venv`는 다른 프로젝트의 Python 패키지 경로를 함께 사용했으므로, 이 임시 환경을 새 환경의 표준 설치 방법으로 가정하지 않는다.

## 2. Gazebo와 카메라 확인

터미널 A에서 실행한다.

```bash
cd /home/hjpark/CLionProjects/robot_simulation
source install/setup.zsh
ros2 launch piper_arm_moveit_config gazebo_moveit.launch.py
```

터미널 B에서 카메라 토픽을 확인한다.

```bash
cd /home/hjpark/CLionProjects/robot_simulation
source install/setup.zsh
ros2 topic list | rg '/left_camera/rgbd/(image|points)'
ros2 topic hz /left_camera/rgbd/image
```

RViz에서 영상을 보려면 **Add → Image → Topic `/left_camera/rgbd/image`**를 선택한다. 기존 [Gazebo 사진](piper_red_target_camera.png)처럼 빨간 상자와 그리퍼가 보이는지 확인한다. 영상이 없다면 Gazebo가 실행 중인지, 브리지와 카메라 토픽이 생성됐는지 먼저 확인한다.

현재 launch는 학습된 빨간 상자 모델도 자동 실행한다. 데이터 수집에는 영향을 주지 않지만, 다른 모델을 수동 시험할 때는 중복 노드가 같은 출력 토픽을 발행하지 않도록 기존 launch의 인식 노드를 끄거나 해당 프로세스를 중지한다.

## 3. 이미지와 라벨 수집

현재 [수집 스크립트](../src/piper_arm_moveit_config/scripts/collect_red_box_dataset.py)는 Gazebo 모델 `red_target_box`를 여러 위치로 옮겨 왼쪽 RGB 카메라 이미지를 저장한다. 빨간색 HSV 영역을 찾아 자동으로 바운딩 박스를 만든다. **색상 분리는 라벨 작성에만 사용**하며, 실행 중 검출은 학습된 YOLO가 수행한다. Gazebo가 실행된 상태에서 터미널 B에서 실행한다.

```bash
python3 src/piper_arm_moveit_config/scripts/collect_red_box_dataset.py \
  --output /tmp/robot_yolo_dataset --positive 140 --negative 20
```

성공하면 `Captured 140 positives, 20 negatives`가 표시된다. 스크립트는 종료 시 상자를 원래 위치 `(0.65, 0.29, 0.10) m`로 돌린다. 이미 일부 이미지가 있으면 그 번호 다음부터 이어서 수집한다. 다만 중단 전에 저장된 데이터가 올바른지 먼저 검사한다.

생성되는 구조는 다음과 같다.

```text
/tmp/robot_yolo_dataset/
├── dataset.yaml
├── images/
│   ├── train/red_0001.jpg, empty_0001.jpg, ...
│   └── val/red_0000.jpg, empty_0000.jpg, ...
└── labels/
    ├── train/red_0001.txt, empty_0001.txt, ...
    └── val/red_0000.txt, empty_0000.txt, ...
```

각 이미지에는 같은 이름의 라벨이 필요하다. 물체가 있는 이미지의 라벨 한 줄은 `class_id center_x center_y width height`이며, 네 좌표는 이미지 크기로 나눈 `0~1` 값이다. 예를 들어 `0 0.5 0.5 0.25 0.2`는 클래스 0 물체가 화면 중앙에 있고 화면 너비의 25%, 높이의 20%를 차지한다는 뜻이다. 픽셀 좌표로 박스를 직접 그렸다면 `center_x=(x_min+x_max)/(2×이미지 너비)`, `width=(x_max−x_min)/이미지 너비`로 변환하고 Y와 높이도 같은 방식으로 계산한다. 빈 장면의 `.txt`는 내용이 비어 있다. `dataset.yaml`에는 데이터 위치와 클래스 이름이 기록된다.

학습 전에 이미지 몇 장과 그에 대응하는 라벨을 직접 확인한다. 상자가 잘리거나, 그림자·로봇 암을 상자로 표시하거나, 바운딩 박스가 실제 물체보다 크게 벗어난 이미지는 고친 뒤 학습한다. 이번 수집 결과는 학습 128장(상자 112, 빈 장면 16), 검증 32장(상자 28, 빈 장면 4)이었다. 이번 검증 이미지는 같은 Gazebo 장면에서 수집됐으므로 독립적인 조명이나 카메라에서의 성능을 평가하지 않는다. `/tmp` 아래 데이터와 학습 결과는 임시 파일이므로 계속 사용할 자료는 별도 위치에 복사해 보관한다.

## 4. YOLOv8s 학습

다음은 새 가상 환경에서 바로 실행할 수 있는 형태다. `YOLO('yolov8s.pt')`는 사전 학습 모델이 로컬에 없으면 다운로드하므로 네트워크가 필요하다. 이번 실제 실행에서는 이미 있던 `/home/hjpark/CLionProjects/ROSMASTER-M1/codes/OrinNano/yahboomcar_ws/src/yahboom_yolov8/yahboom_yolov8/yolov8/weights/yolov8s.pt`를 사용했다. 오프라인 환경이면 해당 경로를 `YOLO(...)`에 지정한다.

```bash
/tmp/piper_yolo_venv/bin/python - <<'PY'
from ultralytics import YOLO

model = YOLO('yolov8s.pt')
model.train(
    data='/tmp/robot_yolo_dataset/dataset.yaml',
    epochs=12,
    imgsz=320,
    batch=8,
    device='cpu',
    workers=0,
    project='/tmp/robot_yolo_train_runs',
    name='red_box',
    exist_ok=True,
    patience=12,
    plots=False,
)
PY
```

`imgsz`는 모델 입력 영상 크기, `batch`는 한 번에 처리할 이미지 수다. 메모리가 부족하면 `batch`를 낮춘다. GPU를 사용할 수 있으면 `device`를 해당 장치로 바꿀 수 있다. 결과는 `/tmp/robot_yolo_train_runs/red_box/`에 저장된다. `weights/best.pt`가 만들어졌는지 확인한다.

```bash
ls -lh /tmp/robot_yolo_train_runs/red_box/weights/best.pt
tail -n 1 /tmp/robot_yolo_train_runs/red_box/results.csv
```

이번 12 epoch 학습의 최종 검증 값은 precision `0.998`, recall `1.000`, mAP50 `0.995`, mAP50-95 `0.988`이었다. 지표만으로 결론 내리지 말고 학습에 사용하지 않은 위치와 빈 화면에서도 직접 시험한다. 반복 수집한 비슷한 사진을 학습과 검증에 나누면 점수가 높게 나올 수 있다.

## 5. ROS 노드용 ONNX 변환

`best.pt`는 학습 가중치이고, C++ 노드에 넣을 파일은 `best.onnx`다. 이 노드는 **YOLOv8 detection, 정사각형 입력, NMS 없는 원본 출력**을 기대한다. 다음 명령은 이번에 사용한 `320 × 320`, 배치 1 설정이다.

```bash
/tmp/piper_yolo_venv/bin/python - <<'PY'
from ultralytics import YOLO

model = YOLO('/tmp/robot_yolo_train_runs/red_box/weights/best.pt')
model.export(format='onnx', imgsz=320, batch=1, nms=False, simplify=True, device='cpu')
PY
```

출력 파일 `/tmp/robot_yolo_train_runs/red_box/weights/best.onnx`의 입력과 출력 모양을 확인한다. 단일 클래스의 이번 모델은 입력 `[1, 3, 320, 320]`, 출력 `[1, 5, 2100]`이었다. `5`는 박스 좌표 4개와 클래스 점수 1개다.

```bash
/tmp/piper_yolo_venv/bin/python - <<'PY'
import onnx

model = onnx.load('/tmp/robot_yolo_train_runs/red_box/weights/best.onnx')
onnx.checker.check_model(model)
for value in [*model.graph.input, *model.graph.output]:
    print(value.name, [dim.dim_value or dim.dim_param for dim in value.type.tensor_type.shape.dim])
PY
```

## 6. 가중치 적용 및 ROS 테스트

빨간 상자 모델을 교체하려면 ONNX를 패키지의 모델 폴더로 복사한다. `--symlink-install`로 빌드한 개발 환경에서는 파일 변경이 설치 경로에 바로 보일 수 있지만, 다른 설치 방식에서는 재빌드 후 다시 시작한다. 실행 중인 인식 노드는 시작할 때 모델을 읽으므로 **파일을 교체한 뒤 launch를 재시작**한다.

```bash
cp /tmp/robot_yolo_train_runs/red_box/weights/best.onnx \
  src/piper_arm_moveit_config/model/red_target_yolov8s.onnx
colcon build --packages-select piper_arm_moveit_config --symlink-install
source install/setup.zsh
ros2 launch piper_arm_moveit_config gazebo_moveit.launch.py
```

[launch 설정](../src/piper_arm_moveit_config/launch/gazebo_moveit.launch.py)은 이 ONNX 경로와 `target_class_id: 0`, `confidence_threshold: 0.5`를 인식 노드에 전달한다. 다른 터미널에서 다음을 확인한다.

```bash
source install/setup.zsh
ros2 topic echo /detected_object/pose --once
ros2 topic echo /detected_object/surface_point --once
ros2 topic hz /detected_object/debug_image
```

RViz에 `/detected_object/debug_image`를 Image 디스플레이로 추가하면 검출 박스와 점수를 볼 수 있다. 이번 테스트에서는 상자 중심이 Gazebo `world`의 `(0.65, 0.29, 0.10) m`에 있을 때 발행된 Pose가 약 `(0.6500, 0.2907, 0.1126) m`이었다. RGB 박스 검출은 성공했지만 중심의 Z가 약 1.3 cm 달랐으므로 3D Pose를 집기 동작에 사용하기 전에는 실제 물체 중심과 비교해야 한다.

### 카메라 영상에 위치·크기 표시하기

인식 노드의 `/detected_object/debug_image`에는 빨간 박스, 검출 점수, 이미지 픽셀 기준 박스의 왼쪽 위 좌표 `x,y`와 폭·높이 `w,h`, 추정한 `world` 중심 위치(미터), **설정된** 물체 크기(미터)가 표시된다. `/detected_object/pose`는 카메라 영상과 포인트클라우드로 추정한 위치다. 물체의 실제 크기는 YOLO가 측정한 값이 아니라 노드 파라미터 `object_size_x_m`, `object_size_y_m`, `object_size_z_m`에 입력한 값이다. 현재 빨간 상자에서는 각 값이 `0.10 m`이다.

Gazebo launch를 실행한 상태에서 다른 터미널에서 [카메라 뷰어 노드](../src/piper_arm_moveit_config/src/red_box_camera_view_node.cpp)를 시작하면 영상 창이 뜨고, 위치·설정 크기·TCP와 물체 사이 거리가 터미널에도 초당 약 한 번 출력된다. 영상 창에는 **TCP에서 물체 중심까지의 직선거리**와 **TCP에서 상자 표면까지의 최단거리**가 미터 단위로 표시된다. 후자는 설정된 상자 크기와 물체 Pose를 이용한 계산값이다.

```bash
cd /home/hjpark/CLionProjects/robot_simulation
source install/setup.zsh
ros2 run piper_arm_moveit_config red_box_camera_view_node --ros-args -p use_sim_time:=true
```

현재 URDF에는 `left_tcp` 프레임이 없으므로 뷰어의 기본 TCP는 `left_gripper_base`에서 로컬 `(0, 0, 0.16) m` 떨어진 **추정 지점**이다. 이 `0.16 m`는 실측값이 아니다. 실제 TCP를 측정했다면 `tcp_offset_x_m`, `tcp_offset_y_m`, `tcp_offset_z_m`을 수정한다. URDF에 `left_tcp` 프레임을 추가했다면 `-p tcp_frame:=left_tcp -p tcp_offset_z_m:=0.0`으로 실행한다. `size_x_m`, `size_y_m`, `size_z_m`도 인식 노드의 물체 크기 설정과 같아야 표면 거리가 맞는다. TCP와 물체 위치가 서로 다른 시점의 값이 되지 않도록 뷰어는 물체 Pose의 타임스탬프로 TF를 조회한다.

뷰어가 발행하는 `/detected_object/camera_view`에는 TCP 거리까지 함께 그려진다. RViz에서 **Add → Image → Topic `/detected_object/camera_view`**를 선택하면 같은 영상을 볼 수 있다. 창을 열 수 없는 환경에서는 뷰어에 `-p show_window:=false`를 추가해 실행한 뒤 이 토픽을 RViz에서 확인한다. 창을 닫으려면 뷰어를 실행한 터미널에서 `Ctrl+C`를 누른다. 검출이 안 되면 영상에 `no detection`이 표시되며 Pose는 발행되지 않는다. 깊이 또는 TF가 없으면 검출 박스는 보이지만 중심 위치나 TCP 거리는 표시되지 않는다.

## 다른 물체로 바꾸기

1. Gazebo에 새 물체를 배치하고 카메라에 충분히 보이는지 확인한다. 실제 카메라를 사용할 경우에는 실제 영상으로 데이터셋을 만든다.
2. 새 물체의 이미지와 정답 박스를 모은다. [기존 수집 스크립트](../src/piper_arm_moveit_config/scripts/collect_red_box_dataset.py)는 `red_target_box`, 빨간색 HSV 범위, 이동 좌표, `/left_camera/rgbd/image`, 복귀 좌표가 고정돼 있다. 새 물체에는 이 부분을 수정하고 라벨 정확도를 검사하거나, 이미지를 저장한 뒤 사람이 바운딩 박스를 표시한다. 색으로 구별하기 어려운 물체는 HSV 자동 라벨을 그대로 사용하면 안 된다.
3. 별도 데이터 폴더를 만들고 `images/train`, `images/val`, `labels/train`, `labels/val`을 구성한다. `dataset.yaml`의 `path`와 `names`를 새 데이터에 맞춘다. 클래스가 하나면 ID 0, 여럿이면 0부터 연속 번호를 준다. 빈 이미지는 학습 대상이 없는 장면으로 포함한다.
4. 4절의 `data`, 출력 `project/name`, epoch 등을 새 작업에 맞춰 학습한다. 검증 이미지뿐 아니라 새로운 위치·거리·조명에서도 오검출과 미검출을 확인한다.
5. 5절처럼 `nms=False`로 ONNX를 내보낸다. [launch 설정](../src/piper_arm_moveit_config/launch/gazebo_moveit.launch.py)의 `model_path`를 새 파일로, `target_class_id`를 찾을 클래스 번호로 바꾼다. 이 노드는 한 번에 최고 점수 검출 하나의 Pose만 발행한다. 여러 물체를 동시에 구분해 발행하려면 노드를 확장해야 한다.
6. 물체의 실제 X축 크기를 `object_size_x_m`에, 월드 기준 방향을 `object_yaw_rad`에 반영한다. 현재 Pose 계산은 카메라에 보이는 표면이 물체 로컬 `−X` 면이라고 가정한다. 모양이나 보이는 면이 다르면 [인식 노드](../src/piper_arm_moveit_config/src/color_object_pose_node.cpp)의 중심 보정 방법을 바꿔야 한다. 2D 검출이 잘돼도 이 가정이 틀리면 3D Pose는 틀릴 수 있다.

새 물체 데이터셋을 손으로 준비할 때는 아래처럼 시작할 수 있다. 촬영한 RGB 사진을 `images/train`과 `images/val`에 나누고, 이미지마다 동일한 이름의 YOLO 형식 `.txt`를 `labels`의 대응 폴더에 만든다. 바운딩 박스를 그려 YOLO 형식으로 내보내는 이미지 주석 도구를 사용해도 된다.

```bash
DATASET_DIR=/tmp/new_object_dataset
mkdir -p "$DATASET_DIR"/images/{train,val} "$DATASET_DIR"/labels/{train,val}
cat > "$DATASET_DIR/dataset.yaml" <<EOF
path: $DATASET_DIR
train: images/train
val: images/val
names:
  0: new_object
EOF
```

예를 들어 `images/train/frame001.jpg`가 `640 × 480`이고 물체 박스의 픽셀 범위가 `x=100~220, y=80~200`이라면 `labels/train/frame001.txt`에는 `0 0.25 0.291667 0.1875 0.25`를 적는다. 새 물체가 없는 `frame002.jpg`에는 빈 `frame002.txt`를 만든다. 이후 4절 명령의 `data`를 `$DATASET_DIR/dataset.yaml`의 **실제 절대 경로**로 바꿔 학습한다.

## 자주 막히는 지점

| 증상 | 확인할 것 |
| --- | --- |
| 수집이 `No fresh left camera RGB image`로 중단됨 | Gazebo와 `/left_camera/rgbd/image`가 살아 있는지, 시뮬레이션이 일시정지되지 않았는지 확인한다. |
| `Gazebo rejected box pose` | 모델 이름 `red_target_box`, 월드 이름 `piper`, `/world/piper/set_pose` 서비스가 맞는지 확인한다. |
| 이미지에 물체가 있는데 라벨이 없음 | 색상 임계값이나 물체 위치가 수집 스크립트의 조건에 맞는지 확인한다. 다른 물체라면 라벨 생성 방법을 바꾼다. |
| 학습 중 Python 모듈 오류 | 학습 가상 환경에 필요한 패키지가 있는지 확인한다. 이번 작업에서는 `polars` 누락으로 첫 epoch 후 중단되어 설치 후 재실행했다. |
| ONNX를 노드가 거부함 | detection 모델인지, NMS 없이 내보냈는지, 입력이 3채널 정사각형인지, 클래스 수와 `target_class_id`가 맞는지 확인한다. |
| 검출 박스는 보이는데 Pose가 없음 | `/left_camera/rgbd/points`와 이미지 크기·시각이 맞는지, 포인트클라우드에 유효한 XYZ가 있는지, 카메라에서 `world`로 가는 TF가 있는지 확인한다. |
| 검증 점수는 높은데 새 장면에서 실패함 | 학습·검증 장면이 너무 비슷할 수 있다. 거리·시점·조명·배경을 다양하게 수집하고 새로운 조건을 별도로 시험한다. |

관련 문서: [TCP 접근 및 물체 Pose 설명](piper_tcp_pregrasp.md), [Gazebo 실행 안내](../src/piper_arm_moveit_config/gazebo_launch.md).
