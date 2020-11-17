# Cooperative Guidance
Cooperative guidance for multiple kill vehicles

code : MATLAB & python

## Simulator with Single Target and Kill Vehicle
본 시뮬레이션 모듈은 단일 표적 격추를 위해 기동하는 한 대의 Kill Vehicle을 모사한다. 표적 및 Kill Vehicle의 운동을 모사하기 위해 각 비행체에게 가해지는 모멘트 및 회전 운동을 직접 고려하는 6DOF 모델을 적용하는 것이 이상적이나, 이는 필요 이상으로 과도한 수준의 충실도를 갖는다고 판단된다. 본 시뮬레이션 모듈에는 비행체의 질점을 모사하는 3DOF 모델을 기반으로, Kill Vehicle의 자세각 변화를 추가적으로 모사하는 Pseudo 6DOF 모델을 적용하였다.

### Simulator Structure
![Simulator Structure](https://user-images.githubusercontent.com/55905711/99349869-56dc5e00-28e0-11eb-934a-3b9e1a718467.png)


사용자는 시뮬레이션 시작 전 표적 및 Kill Vehicle의 초기 위치, 진행방향, 속력 등을 입력한다. 시뮬레이션 모듈은 입력받은 초기 파라미터를 기반으로 표적 궤적을 적분하며, Kill Vehicle은 생성된 표적 궤적을 기반으로 [비례항법유도(Proportional Navigation Guidance)](https://en.wikipedia.org/wiki/Proportional_navigation)를 실시, 표적을 향해 기동한다. Kill Vehicle과 표적 간의 접근속도가 양수에서 음수로 변하면 표적이 격추되었다고 가정, 시뮬레이션을 종료하고 궤적 및 속도 데이터를 출력한다. 

---

### Target Trajectory
![Target Flight Angle](https://user-images.githubusercontent.com/55905711/99188873-92bcd980-27a1-11eb-9acf-46bd82e86da2.png)

표적은 Kill Vehicle과 독립적으로 기동하며, 사용자에 의해 기동 시점 및 수직, 수평방향 기동 각속도를 입력받는 3DOF 모델을 따른다. 시뮬레이션 모듈은 입력된 기동 파라미터를 바탕으로 표적의 기동 명령을 생성하며, 생성된 기동 명령은 1차 동역학 시스템을 거쳐 표적의 기동 각도를 산출한다. 계산된 표적의 수직, 수평방향 기동 각도는 각각 *Flight Path Angle*과 *Heading Angle*에 해당한다.

![FPA and Heading](https://user-images.githubusercontent.com/55905711/99349505-85a60480-28df-11eb-87f1-e1f5f402781e.png)

이를 기반으로 표적의 Wind 좌표계를 정의한다. Wind 좌표계는 NED 좌표계를 3축-2축 순서로 Heading Angle, Flight Path Angle만큼 회전한 좌표계로, 비행체의 진행 방향, 즉 속도벡터가 Wind 좌표계의 1축과 일치하게 된다. *Flight Path Angle*과 *Heading Angle*을 이용해 *Wind to NED* 또는 *NED to Wind* 회전변환 행렬을 기술하며, 표적의 속력을 특정한다면 기동하는 표적의 속도 및 궤적을 NED 좌표계에서 나타낼 수 있다. 

---

### Proportional Navigation Guidance
---
![proportionalNavigation](https://user-images.githubusercontent.com/70247735/99353931-d79f5800-28e8-11eb-9db1-1e7ccef7077f.png)

비례 항법 유도라고 불리는 Proportional Navigation Guidance의 개념을 설명하기 위한 2차원 그림은 위와 같다. Proportional Navigation Guidance는 위 그림에서 나타나는 *line of sight rate*(람다 dot)이 0이 되도록  Kill Vehicle를 유도하는 방법이며, Kill Vehicle과 target의 이동 경로를 시각화한 삼각형을 *collison triangle* (충돌 삼각형) 이라고 한다.<br>

##### 2차원
2차원에서의 Kill Vehicle의 가속도는 다음과 같이 나타낼 수 있다.<br>
  (수식)<br>
위의 수식에서 알 수 있듯이, Kill Vehicle의 가속도는 Kill Vehicle의 순간 속도 벡터에 수직이며, 이때 *N*은 무차원의 비례상수, ( 람다 dot )은 *line of sight rate*, V는 *closing velocity*이다.<br>

##### 3차원
3차원에서의 표준 Kill Vehicle 가속도는 다음과 같이 나타낼 수 있다.<br>
  (수식)<br>
위 수식에서 *N*는 무차원의 비례상수이고, V는 *Kill Vehicle대한 target 속도*이다. (오메가)는 *line of sight의 rotation vector*이며, 다음과 같이 나타낼 수 있다.<br>
(로테이션 벡터 수식)<br>

3차원에서 Kill Vehicle의 가속도는 Kill Vehicle의 속도 벡터와 *R*(range from Kill Vehicle to target)에 대해 나타낼 수 있으며, 다음과 같이 나타낼 수 있다.<br>
  (수식)<br>
  (수식)
  
![수식](http://latex.codecogs.com/gif.latex?&theta;%3D%5Cfrac%7By%7D%7Bx%7D)

### Kill Vehicle Kinematics and Dynamics

---
