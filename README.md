# [ROS] BLE 페리페럴 & 아이 트래킹

본 저장소는 ROS(Robot Operating System)위에서 **BLE 페리페럴**과 **아이 트래킹** 부분이 통합된 저장소입니다.

아이 트래킹은 아래의 저장소에서 개발된 후, 이곳에서 통합되었습니다.<br/>
(readme는 본 저장소가 최신이므로 아래의 저장소 readme는 확인하지 않으셔도 됩니다.)

[HUDuck 아이 트래킹 바로가기 - GitLab](https://git.swmgit.org/swm-12/12_swm35/eye-tracking)

# 1. BLE 페리페럴 (Bluetooth Low Energy Peripheral)

사용자 스마트폰과 통신을 위해 HUD 디바이스에 BLE(Bluetooth Low Energy) Peripheral 역할을 부여하였습니다.

스마트폰에서 블루투스를 통해 넘어온 데이터는 가공되어 ROS상의 Subscriber에게 줄 수 있는 형태의 Message로 전달됩니다.

## 1.1. Data Interface Descriptions

<img src="https://git.swmgit.org/swm-12/12_swm35/applicationreceiver/-/raw/readme/readme_src/Peripheral%20Data%20Interface%20Description.png">

## 1.2. 실행 환경 구성

### 1.2.1. 블루투스 설정값 변경

아래의 명령어를 사용하여 블루투스 통신에 필수적인 기능을 켜줘야 합니다.

```
sudo btmgmt -i hci0 power off;

sudo btmgmt -i hci0 bredr off;
sudo btmgmt -i hci0 privacy off;
sudo btmgmt -i hci0 hs off;
sudo btmgmt -i hci0 sc off;
sudo btmgmt -i hci0 ssp off;
sudo btmgmt -i hci0 linksec off;
sudo btmgmt -i hci0 pairable off;
sudo btmgmt -i hci0 fast-conn off;
sudo btmgmt -i hci0 bondable off;

sudo btmgmt -i hci0 le on;
sudo btmgmt -i hci0 advertising on;
sudo btmgmt -i hci0 connectable on;
sudo btmgmt -i hci0 discov on;

sudo btmgmt -i hci0 power on;
```

## 1.3. 시연 영상

[![시연 영상](http://img.youtube.com/vi/qQFVhA9BBH8/0.jpg)](https://youtu.be/qQFVhA9BBH8)

# 2. 아이 트래킹 (Eye Tracking)

HUDuck의 HUD는 아이 트래킹을 지원합니다.

아이 트래킹을 통해 운전자 눈 위치를 계산하여 HUD의 정확한 위치에 그래픽을 그려줍니다. 

## 2.1. 사용 이유

화물차에 설치된 에어 서스펜션 시트는 운전자 눈 위치를 계속적으로 변화시킵니다.

이는 AR HUD가 운전자 눈에 정확한 상을 전달하지 못하도록 방해합니다.

이를 극복하기 위해, 연속적으로 운전자 눈 위치를 계산하고 HUD에 표시될 그래픽의 위치를 보정해줍니다.

## 2.2. 아이 트래킹 처리 과정

### 2.2.1. Perspective Transform(원근 변환) 적용

원근 변환을 통해 운전자를 정면에서 보는 시점으로 전환합니다.

캘리브레이션 과정에서 사용된 판의 정확한 크기를 알고 있기 때문에, 픽셀 당 실제 거리를 알 수 있습니다.

### 2.2.2. 얼굴 인식

우선, 영상을 회색조로 변환 및 크기 축소(1/10 크기)를 거칩니다.

Haar Cascade 알고리즘을 사용하여 단순화된 영상에서 얼굴을 인식합니다.

### 2.2.3. 얼굴 특징점 추출

원본 크기의 회색조 영상에서 구해진 얼굴 영역을 기반으로 5개의 얼굴 특징점을 추출합니다.

5개의 특징점 추출은 일반적으로 사용되는 68개의 특징점 추출보다 약 8~10% 더 빠른 처리 속도를 보입니다.

### 2.2.4. 눈 위치  지정

특징점 중 눈의 시작점과 끝점을 연결하고, 이 선의 중점을 눈의 중심으로 정의합니다.

두 눈의 중심을 연결하고, 이 선의 중점을 눈 위치라고 정의합니다.

### 2.2.5 눈 위치 변화 계산

눈 위치 변화(px)에 픽셀 당 실제 거리를 곱하여 눈이 이동한 거리를 계산할 수 있습니다.

기준 눈 위치를 기준으로 눈 위치 변화량을 계산하여 ROS 위에서 실행되는 노드에 해당 값을 전달합니다.

## 2.3. 아이 트래킹 Hot key

아이 트래킹의 개발 편의 및 사용을 위해서 여러 기능을 사용할 수 있는 Hot key를 아래와 같이 지정하였습니다.

`q`: 아이 트래킹 종료<br/>
`p`: 아이 트래킹 출력 on/off (기본: on)<br/>
`s`: 눈 추적 영역 설정 모드 on/off (기본: off)<br/>
`r`: 눈 추적 영역 설정 모드 일시정지 on/off (기본: off) (영역 설정 모드가 on일 경우 사용 가능)<br/>
`e`: 기준 눈 위치 설정 (3초간 눈 위치 평균 값)<br/>
`t`: 기준 눈 위치 설정 (현재 눈 위치)

## 2.4. 시연 영상

원근 변환부터 ROS 실행까지 모든 과정을 담은 시연 영상은 아래에서 보실 수 있습니다.

[![시연 영상](http://img.youtube.com/vi/9YfRtpptQkw/0.jpg)](https://youtu.be/9YfRtpptQkw)

# 3. 프로젝트 시작하기

 프로젝트 폴더 최상위에 있는 `start.sh` 파일을 실행하면 자동으로 블루투스 설정과 노드가 실행됩니다.
