# py_bt_ros

ROS 2 기반 Behavior Tree(BT) 프레임워크로 로봇 내비게이션, YOLO 객체 탐지, 그리고 커스텀 UI를 통한 미션 제어를 통합한 프로젝트입니다.

---

# Overview
이 저장소는 로봇 제어를 위한 BT 엔진을 제공하며 아래 기능을 통합합니다.

- **ROS 2 Integration**: 로봇 하드웨어 및 센서와 통신
- **Navigation**: 내비게이션 스택 연동을 통한 자율 이동
- **YOLO Object Detection**: 실시간 사람(Person) 탐지 기반 `Flee` / `Scan` 동작 지원
- **Camera Service**: 전용 카메라 모듈 기반 이미지 캡처 및 저장 관리
- **Stop Button UI (PyGame)**: 수동 미션 중단/상태 제어 GUI
- **Custom ROS 2 Actions**: `RunOpposite` 등 커스텀 Action 제공  
  - Action 인터페이스는 본 저장소의 **`rb_interface`** 패키지에 포함되어 있습니다.

---

# Build

cd ~/py_bt_ros
colcon build 
source install/local_setup.bash

# 전체 시스템은 총 5개의 터미널에서 실행합니다.
# 각 터미널에서 실행 전 워크스페이스 환경을 먼저 적용하세요



# Terminal 1

ros2 launch wego teleop_launch.py

# Terminal 2 

ros2 launch wego navigation_diif_launch.py

# Terminal 3 


cd ~/py_bt_ros/modules
source install/local_setup.bash
python3 yolo_detector.py

# Terminal 4

cd ~/py_bt_ros/servers
source install/local_setup.bash
python3 run_opposite_server.py

# Terminal 5

cd ~/py_bt_ros
source install/local_setup.bash
python3 main.py
