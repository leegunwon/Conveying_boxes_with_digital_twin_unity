# 디지털 트윈을 이용한 박스 컨베이어 시스템 (Unity)

## 개요
본 프로젝트는 **PyQt5** 기반의 GUI를 통해 명령을 입력받아, **Aruco marker**을 통해 주행하고 **YOLO v8s** 모델을 활용하여 박스를 탐지하고 **manipulator**와 **conveyor**으로 이동시켜 자동으로 물건을 분류하는 시스템입니다.

위 과정은 **Unity**로 구축한 디지털 트윈 환경에서 모니터링 가능하도록 구현되어 있습니다.


## Project Member
이건원, 김화림, 이대원, 이수형


## 주요 기능

- **디지털 트윈**: **Unity를 통해** 컨베이어 시스템, Turtlebot waffle, manipulation X의 움직임을 가상환경에서 관찰.
- **ROS 2 통합**: Rosbridge를 통해 Unity와 ROS 2 간의 데이터 전달이 이루어짐.
- **컨베이어 시스템**: **serial 통신**을 통해 아두이노 컨베이어 벨트를 동작시켜 박스를 이동시키는 과정.
- **marker tracing**: **Aruco marker**를 통해 Turtlebot을 원하는 위치로 이동시키는 동작을 수행.
- **Manipulator 제어**: **Moveit 패키지**를 활용해 **Manipulator X**를 제어.
- **Vision을 활용한 제어** : 프로젝트에 사용될 박스와 basket에 있는 mark를 학습시킨 **YOLO v8s**모델을 활용하여 거리를 측정한 후 물체를 Pick up 하는 동작을 수행.


## 환경 사진
### Unity
![스크린샷 2025-02-28 14-02-04](https://github.com/user-attachments/assets/e000dd81-bd40-42d4-81c5-76c17863ab7d)

### Real
![20250228_154319](https://github.com/user-attachments/assets/dbf3ab2b-87a9-49e5-9672-850845ecfd93)


### 필수 설치 사항

다음 항목들이 설치되어 있어야 합니다:

1. **Unity** (6000.0.39f1)
2. **ROS 2** (Humble)
3. **Python 3.8 이상**
4. **PyQt5**
5. **openCV**
7. **ultraystics**

## 실행 방법
### 로봇
1. turtlebot3_manipulation, aruco_and_yolo_detection 패키지를 turtlebot waffle에 옮김
2. cd Conveying_boxes_with_digital_twin_unity
3. . execute_system_robot.sh

### 컴퓨터
1. cd Conveying_boxes_with_digital_twin_unity
2. . execute_system_computer.sh True

## 레퍼지토리 구조
```
📦 **Conveying_boxes_with_digital_twin_unity**  
├── 📂 **src**                 # ROS 2 패키지 소스 코드  
│   ├── 📦 **aruco_and_yolo_detection**    # Aruco 마커 및 YOLO 탐지 패키지  
│   ├── 📦 **conveyor_system_gui**         # PyQt5 기반 GUI 패키지  
│   ├── 📦 **conveyor_controller**         # 아두이노와 Serial 통신 (컨베이어 벨트 제어) 
│   ├── 📦 **robot_control**               # Turtlebot waffle과 Manipulator를 제어
│   ├── 📦 **turtlebot_moveit**            # MoveIt을 활용한 Manipulator 제어 인터페이스   
│   ├── 📦 **turtlebot_cosmo_interface**   # 커스텀 srv
│   └── 📦 **turtlebot3_manipulation**     # Turtlebot waffle과 Manipulation X가 결합된 로봇 실행 
│  
├── 📄 execute_system_computer.sh          # 컴퓨터에서 실행 명령
├── 📄 execute_system_robot.sh             # 로봇에서 실행 명령
└── 📄 README.md 
'''
