# ros-repo-2
파이널 프로젝트 2조 저장소. AI 자율주행 물류 로봇 시스템

<br />
<p align="center">
  <a href="https://github.com/addinedu-ros-8th/deeplearning-repo-5">
    <img src="https://github.com/user-attachments/assets/cf258d3d-4bb2-4909-8fa8-9330b97f4dea" alt="Logo" width="620px">
  </a>

  <h3 align="center">SUGOI (Scan & Understand Goods On the go with Intelligence)</h3>
  <p align="center">
    <a href="https://www.miricanvas.com/v2/design/14nnz4f">Presentation</a>
    <a> || </a>
    <a href="https://youtu.be/28lv0X8Z07Q">Video Demo</a>
  </p>
</p>
<hr>

> **스고~이 봇** 팀이 개발한, AI 자율주행 물류 로봇 시스템입니다. 

---

## 프로젝트 개요

AI 기반 자율주행 로봇 시스템은 차량 번호판 인식과 예약 정보를 활용해 작업을 생성하고 로봇 상태를 실시간으로 감지해 효율적으로 작업을 분배하며,
SLAM & Navigation 기반의 자율주행 기능과 3D 프린터로 제작한 Forklift 구조를 통해 제품 입출고 및 창고 정리 임무를 수행하고
TCP 통신, ROS2, AI 비전, 데이터베이스 연동 등 다양한 기술을 통합해 실제 산업 환경을 모델링한 자동화 시나리오를 구현했습니다.

- **프로젝트명**: SUGOI (Scan & Understand Goods On the go with Intelligence)
- **팀명**: 스고~이 봇
- **주제**: AI 자율주행 물류 로봇 시스템
- **핵심 기술**: ROS2, Python, MySQL, YOLO, OpenCV, Arduino, Raspberry Pi 5, PyQt6, Inventor, ESP32, TCP/UDP

## 팀 구성 및 역할
|        | Name | Job |
|--------|------|-----|
| Leader | 강주빈 |  Project Manage, SLAM & Navigation, 3D Modeling |   
| Worker | 이상윤 |  Forklift Control, Posture Correction |   
| Worker | 이우재 |  Task Manager, DataBase, AI Camera, Plate Camera |    
| Worker | 임동욱 |  SLAM & Navigation, GUI | 

---

## 기술 스택 (Tech Stack)
| Category | Technology |
|----------|------------|
| Development Environment	| Linux, Ubuntu 24.04 LTS, ROS2 (Jazzy), Python venv, Docker |
| Language | Python, C++, MySQL |
| Framework |	Nav2, PyQT6, OpenCV, YOLO, Inventor |
| Network |	TCP/IP, UDP/IP |
| Configuration Management | Github, Jira, Confluence, Slack |


---

## 시스템 아키텍처 구성

#### HW Architecture
![Image](https://github.com/user-attachments/assets/90a1b9ad-eb19-43b3-b0ed-2dd525f51feb)
<br >

#### SW Architecture
![Image](https://github.com/user-attachments/assets/3858b864-0a5f-4eef-bddf-90e1e176b574)
<br >

## Data Structure
![Image](https://github.com/user-attachments/assets/18472ab5-ce5e-44e6-b9dc-4a40a838a5f1)
![Image](https://github.com/user-attachments/assets/6f9bea98-6a64-4ce4-8920-9a5776377967)
<br >

## Interface Specification
![Image](https://github.com/user-attachments/assets/663e6729-1c01-45e0-8ed6-eac7cfe5b564)
![Image](https://github.com/user-attachments/assets/6d63a274-50b8-44fb-9d57-75c0f60c0edb)
<br >

## Sequence Diagram
![Image](https://github.com/user-attachments/assets/1d90afd6-b776-49ad-8bb0-fb38220c83f9)
![Image](https://github.com/user-attachments/assets/4cdde570-6fcc-49cb-adf0-ad3692e3b30d)
![Image](https://github.com/user-attachments/assets/29651e09-31fa-41b5-872c-eca993b8387d)
![Image](https://github.com/user-attachments/assets/7bc228e9-2173-4367-bced-3233fac6682a)
![Image](https://github.com/user-attachments/assets/2619f171-b589-479d-bcf3-9a709832b425)
![Image](https://github.com/user-attachments/assets/a4ad4394-6a57-46a1-81fb-90171af8f232)
<br >

## Map
![Image](https://github.com/user-attachments/assets/b6f3a691-b908-44e5-a653-efe86241694e)
<br >

## 3D Modeling
![Image](https://github.com/user-attachments/assets/73f587c8-09fb-47fd-91a9-d8f5824e4e8c)
<br >

## Implements
## SLAM & Navigation
![Image](https://github.com/user-attachments/assets/36412e04-fca1-464b-9f82-f382e763e5d1)
<br >

## YOLOv8 & EasyOCR
![Image](https://github.com/user-attachments/assets/b496ef4b-9291-4eb9-99a8-be7b11df326b)
<br >

## GUI 
### Reservation GUI
![Image](https://github.com/user-attachments/assets/51091fdb-7486-4647-b12c-cab415a19f2e)
![Image](https://github.com/user-attachments/assets/753ddd0a-bbd9-484f-9e75-5d76d7b34a70)
<br >

### Client GUI
![Image](https://github.com/user-attachments/assets/0eab758f-20e4-46a8-a0b6-c2d105964daa)
![Image](https://github.com/user-attachments/assets/7479aaab-7824-43bf-93d6-ff687282893d)
![Image](https://github.com/user-attachments/assets/9b3d5040-9d15-46fb-9d04-99bf5a5df8f0)
![Image](https://github.com/user-attachments/assets/d78210fe-3360-4c55-9f4c-0d585e5f3c1a)
<br >

### Admin GUI
![Image](https://github.com/user-attachments/assets/39e0674e-6301-4e38-ab3a-57b451aefea3)
<br >

## <a href="https://youtu.be/jC3W9LoOUPE">Video Demo</a>
![Image](https://github.com/user-attachments/assets/63945c7f-ece9-443b-8805-f49219d1f175)
<br >

## Project Schedule
Project Period: 2025.04.09~2025.05.27
![Image](https://github.com/user-attachments/assets/1665d7f7-276e-4f50-85d9-682a49d5ab33)
<br >
