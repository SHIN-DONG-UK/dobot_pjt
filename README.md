# 스마트 TV 제조 공정 모사 프로젝트

<img src="./resource/images/시연.gif">

## 과제 개요
- 자동화된 스마트 TV 생산 라인을 모사한다.
- **pick and place** - Dobot을 활용해 부품을 컨베이어 벨트 위에 올려놓는다.
- **비전 인식+제어** - 부품이 벨트 위에 올라오면, 비전 인식으로 back panel과 board panel을 구분하여 sorter를 제어한다.
- ~~이후 공정은 RoboDK 가상 환경에서 구축하고 둘 사이를 연계한다.~~
- 로봇 상태를 웹 패널에 띄운다.

## 개발 환경
- **운영 체제**: Ubuntu 22.04 (서버), Raspberry Pi OS (컨베이어)
- **프로그래밍 언어**: Python 3.10
- **주요 하드웨어**:
  - Dobot Magician (로봇)
  - Intel RealSense D435i (카메라)
  - Raspberry Pi (컨베이어 제어)
- **주요 소프트웨어**:
  - Yolov5 (객체 탐지 모델)
  - ROS2 (로봇 제어 및 데이터 통신)
  - ~~RoboDK (시뮬레이션)~~
  - ~~Django (백엔드 서버)~~
  - ~~Vue.js (프론트엔드)~~


## 구조도
<img src="./resource/images/구조도.png">

## 주요 구현 사항
### 1. Dobot Client
<img src="./resource/images/로봇티칭.png" width=50%>

- 원하는 동작의 웨이포인트들을 기록

<img src="./resource/images/dobot_control_node.png" width=50%>

- PTP action과 suction cup service 제어 함수 작성

<img src="./resource/images/main_loop.png" width=50%>

- main loop 작성

### 2. 비전 인식
<img src="./resource/images/비전인식.png">

- 직접 영상 데이터 취득 후 RoboFlow로 라벨링 작업
- 데이터 증강 후 인식 Yolov5 Object Detection 모델 추출

### 3. Camera Node
<img src="./resource/images/Camera.png">

- pyrealsense2 라이브러리를 통해 realsense의 영상 취득
- yolov5 모델을 통과시켜 인식 결과 추출
- 인식된 경우 그 결과에 따라 socket 통신으로 라즈베리 파이(컨베이어 벨트)에 전송
- 인식 결과를 포함한 영상 publish

### 4. 로봇 티칭
<img src="./resource/images/teaching.png">

- magician_ros2 패키지를 활용해 로봇의 현재 관절값 추출
- 사용자 입력을 대기하고 있다가, 그 순간의 dobot position을 csv 파일로 기록
- 키보드 입력을 threading으로 처리하여 동시성 확보

