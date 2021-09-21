# Project4
프로그래머스 자율주행 데브코스 시뮬레이터 SLAM 프로젝트

## Video
---


## Goal
---
![map](./image/map.png)
- 차선을 벗어나지 않고 3바퀴 연속 주행

## Environment
---
- Ubuntu 18.04
- ROS Melodic
- Xycar 무슨 모델인지 까먹음 ㅎㅎ
- Nvidia TX 2

## Structure
---
~~~
Project4
  └─ map
  │    └─ path_youngjin_last.pkl             # reference path
  └─ path_follower
  │    └─ src             
  │        └─ ego_vehicle.py              # make vehicle go straight
  │        └─ map_drawer.py               # draw reference path on rviz
  │        └─ stanley.py                  # calculate yaw_term, cte_term to return angle
  │        └─ stanley_follower.py         # Publish motor topic through angle value and fixed speed value calculated in stanley.py
  │        └─ stanley_follower_v2.py      # Publish the motor topic by obtaining the angle value and the current speed value calculated in stanley.py
  └─ xycar_imu   #A package that allows publishing to imu topics.
  └─ xycar_msgs  #A package that allows publishing to motor topics.
  └─ xycar_slam
  │    └─ config             # lua file
  │        └─ localization_xytron_without_imu_youngjin.lua           
  │    └─ launch        #launch file
  │        └─ unity_localization_with_stanley.launch        
  │    └─ maps   # mapped map
  │        └─ comp1_youngjin_last_without_imu.pbstream
  │    └─ rviz              # rviz file
  │        └─ localization.rviz
  │        └─ mapping.rviz
  │    └─ urdf    # urdf file
  │        └─ xycar.urdf
~~~

## Usage
---
~~~bash
$ roslaunch xycar_slam unity_localization_with_stanley.launch
~~~

## Procedure & Try
---
### mapping
![image (1)](https://user-images.githubusercontent.com/65532515/134115059-b5a23b5b-6c2d-4ff0-afbf-30df73460156.png)
- mapping 전용 lua 파일을 이용하여 주행할 맵을 매핑한다.
### localization
![image](https://user-images.githubusercontent.com/65532515/134115078-2363dcf3-bf48-4583-8581-e31ba3d6c4bd.png)
- 위에서 제작한 맵을 토대로 localization을 진행한다. 
### path planning & steering control
![image](https://user-images.githubusercontent.com/65532515/134119319-62f924a7-be56-4271-8923-5a333136f601.png)
- 맵의 x, y 좌표와 차의 현재 위치(x, y)좌표를 비교하여 heading error와 cross track error(cte) 를 구하고, steering angle 값을 도출하여 reference path대로 따라갈 수 있도록 path planning 진행.

## Limitations
---
- stanley method는 x, y좌표가 rear wheel기준인데, reference path의 x, y값은 시뮬레이터 상에서 자동차의 앞부분에 라이다가 달려있다고 가정하고 기록된 좌표이기 때문에 둘 사이의 간극을 메울 필요가 있었음 
  - front_x, front_y 값을 현재 x, y값에 자동차의 L(wheel base) * cos(yaw), L(wheel base) * sin(yaw)값 만큼 더해준 값을 사용하였다.
- 코너구간에서 조향을 하지 않고 엉뚱한 곳에서 조향을 하는 문제가 있었음.
  - 발행하는 모터 토픽값은 실제 자동차의 속도와 차이가 있었음. ((이동한 거리 / (x, y)좌표 발행주기) 만큼을 속도로 하여 stanley method의 cte term을 구할 때 사용.

## What I've learned
---
- 발행하는 모터 토픽 값과 실제 자동차의 속도는 차이가 있다는 사실을 깨달았고, stanley error를 구할 때 속도가 들어가는 method인 만큼 토픽값을 현실과 맞추는 과정이 필요함을 깨달음.
- path planning과 tracking 하는 과정을 시뮬레이터를 통해 직접 구현해보는 경험을 얻음.
- SLAM
