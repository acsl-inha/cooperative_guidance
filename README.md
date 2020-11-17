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

비례 항법 유도라고 불리는 Proportional Navigation Guidance의 개념을 설명하기 위한 2차원 그림은 위와 같다. Proportional Navigation Guidance는 위 그림에서 나타나는 *line of sight rate*(&lambda; *dot*)이 0이 되도록  Kill Vehicle를 유도하는 방법이며, Kill Vehicle과 target의 이동 경로를 시각화한 삼각형을 *collison triangle* 이라고 한다.<br>

##### 1) 2차원
2차원에서의 Kill Vehicle의 가속도는 다음과 같이 나타낼 수 있다.<br>

<img src="https://latex.codecogs.com/svg.latex?\Large&space;a_m=N\dot{\lambda}V" title="a_m=N\dot{\lambda}V" />
 
위의 수식에서 알 수 있듯이, Kill Vehicle의 가속도 *a<sub>m*은 Kill Vehicle의 순간 속도 벡터에 수직이며, 이때 *N*은 무차원의 비례상수, &lambda; *dot*은 *line of sight rate*, *V*는 *closing velocity*이다.<br>

##### 2) 3차원
3차원에서의 Kill Vehicle 가속도의 기본형은 다음과 같이 나타낼 수 있다.<br>

<img src="https://latex.codecogs.com/svg.latex?\Large&space;\vec{a}_m=N\vec{V}_r\times\vec{\Omega}" title="\vec{a}_m=N\vec{V}_r\times\vec{\Omega}" />
  
위 수식에서 *N*는 무차원의 비례상수이고, *V*는 *Kill Vehicle대한 target 속도*이다. &Omega;는 *line of sight의 rotation vector*이며, 다음과 같이 나타낼 수 있다.<br>

<img src="https://latex.codecogs.com/svg.latex?\Large&space;\vec{\Omega}={\vec{R}\times\vec{V}_r\over\vec{R}\cdot\vec{R}}" title="\vec{\Omega}={\vec{R}\times\vec{V}_r\over\vec{R}\cdot\vec{R}}" /><br>

3차원에서 Kill Vehicle의 가속도는 Kill Vehicle의 속도 벡터와 *R vector*(range from Kill Vehicle to target)에 대해 나타낼 수 있으며, 다음과 같이 나타낼 수 있다.<br>

  <img src="https://user-images.githubusercontent.com/70247735/99411283-c2e9b100-2936-11eb-8f77-a9abd9869c6b.png" width="270" height="90">
  
 <img src="https://user-images.githubusercontent.com/70247735/99411352-da289e80-2936-11eb-87e6-32407a6dc7ce.png" width="270" height="90">


---

### Kill Vehicle Kinematics and Dynamics

---

### Divert Attitude Control System

![Kill Vehicle Graphics](https://user-images.githubusercontent.com/55905711/99368419-76827f00-28fe-11eb-9936-9bc703d17989.png)

본 연구에서 고려하는 Kill Vehicle은 [RIM-161 Standard Missile 3](https://en.wikipedia.org/wiki/RIM-161_Standard_Missile_3)의 [Kill Vehicle](https://youtu.be/GGvlNufdeL8)과 유사한 형태를 가지고 있다고 가정한다. 동체 전면부에는 표적 포착을 위한 탐색기가 위치하며, 질량중심 주변에 4개의 추력기로 구성된 Divert Control System이, 동체 후면부에 6개의 추력기로 구성된 Attitude Control System이 부착된 형태이다.

![Kill Vehicle Thrusters](https://user-images.githubusercontent.com/55905711/99400646-8adc7100-292a-11eb-82d2-eb7e8344d26e.png)

Kill Vehicle에 부착된 각 추력기들을 위와 같이 나타내었다. 여기서 *D<sub>1</sub>*, *D<sub>2</sub>*, *D<sub>3</sub>*, *D<sub>4</sub>* 와 *A<sub>1</sub>*, *A<sub>2</sub>*, *A<sub>3</sub>*, *A<sub>4</sub>*, *A<sub>5</sub>*, *A<sub>6</sub>* 는 각 DCS 및 ACS 추력기들이 발생시키는 추력울 의미하며, *f<sub>y</sub>*, *f<sub>z</sub>* 는 동체 좌표계의 2축, 3축 방향으로 Kill Vehicle의 질량중심에 작용하는 힘을, 그리고 *l*, *m*, *n* 은 Kill Vehicle의 자세각, 즉 *Roll*, *Pitch*, *Yaw* 를 발생시키기 위한 토크를 나타낸다.

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?\Large&space;\begin{align*}f_y&=-D_2+D_4-A_2-A_3+A_5+A_6\\f_z&=D_1-D_3+A_1-A_4\\l&=-bA_2+bA_3-bA_5+bA_6\\m&=aA_1-aA_4\\n&=aA_2+aA_3-aA_5-aA_6\end{align*}" />  

Kill Vehicle에 작용하는 힘과 토크는 다음과 같이 나타난다. 6개의 ACS 추력기들이 동체 후면부에만 존재하므로, 그 위치는 질량중심에서 크게 벗어난 동시에 질량중심 기준에서 비대칭을 이룬다. 따라서 자세제어를 위해 ACS 추력기를 작동시키는 순간 Kill Vehicle은 토크 *l*, *m*, *n* 뿐만 아니라 힘 *f<sub>y</sub>*, *f<sub>z</sub>* 를 동시에 받을 것이다. *f<sub>y</sub>*, *f<sub>z</sub>* 를 기술하는 식에 ACS 추력기에 의한 *A<sub>1</sub>*, *A<sub>2</sub>*, *A<sub>3</sub>*, *A<sub>4</sub>*, *A<sub>5</sub>*, *A<sub>6</sub>* 항이 포함된 것은 이 때문으로, 동체 전면부에 *f<sub>y</sub>*, *f<sub>z</sub>* 를 상쇄할 수 있는 별도의 추력기가 질량중심 기준 대칭점에 존재하지 않는 한 불가피한 문제이다. *l*, *m*, *n* 을 기술하는 식은 상대적으로 간단하나, ACS 추력기들에 의한 추력을 토크로 변환하는 과정에서 설계변수 *a*, *b*, 즉 질량중심과 ACS 추력기들 사이의 거리 정보가 사용되었다.

---
