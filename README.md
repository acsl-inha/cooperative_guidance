# Cooperative Guidance
Cooperative guidance for multiple kill vehicles

code : MATLAB & python

## Simulator with Single Target and Kill Vehicle
본 시뮬레이션 모듈은 단일 표적 격추를 위해 기동하는 한 대의 Kill Vehicle을 모사한다. 실행시간의 단축을 위해 표적은 3DOF 모델이 적용된 질점으로 가정하며, Kill Vehicle에는 비행체의 질점과 자세각 변화를 모사하는 6DOF 모델이 적용되었다.

### Simulator Structure
![Simulator Structure](https://user-images.githubusercontent.com/55905711/99349869-56dc5e00-28e0-11eb-934a-3b9e1a718467.png)


사용자는 시뮬레이션 시작 전 표적 및 Kill Vehicle의 초기 위치, 진행방향, 속력 등을 입력한다. 시뮬레이션 모듈은 입력받은 초기 파라미터를 기반으로 표적 궤적을 적분하며, Kill Vehicle은 생성된 표적 궤적을 기반으로 [비례항법유도(Proportional Navigation Guidance)](https://en.wikipedia.org/wiki/Proportional_navigation)를 실시, 표적을 향해 기동한다. Kill Vehicle과 표적 간의 접근속도가 양수에서 음수로 변하면 표적이 격추되었다고 가정, 시뮬레이션을 종료하고 궤적 및 속도 데이터를 출력한다. 

---

### Target Trajectory
![Target Flight Angle](https://user-images.githubusercontent.com/55905711/99188873-92bcd980-27a1-11eb-9acf-46bd82e86da2.png)

표적은 Kill Vehicle과 독립적으로 기동하며, 사용자에 의해 기동 시점 및 수직, 수평방향 기동 각속도를 입력받는 3DOF 모델을 따른다. 시뮬레이션 모듈은 입력된 기동 파라미터를 바탕으로 표적의 기동 명령을 생성하며, 생성된 기동 명령은 1차 동역학 시스템을 거쳐 표적의 기동 각도를 산출한다. 계산된 표적의 수직, 수평방향 기동 각도는 각각 Flight Path Angle과 Heading Angle에 해당한다.

![FPA and Heading](https://user-images.githubusercontent.com/55905711/99349505-85a60480-28df-11eb-87f1-e1f5f402781e.png)

이를 기반으로 표적의 Wind 좌표계를 정의한다. Wind 좌표계는 NED 좌표계를 3축-2축 순서로 Heading Angle, Flight Path Angle만큼 회전한 좌표계로, 비행체의 진행 방향, 즉 속도벡터가 Wind 좌표계의 1축과 일치하게 된다. Flight Path Angle과 Heading Angle을 이용해 Wind to NED 또는 NED to Wind 회전변환 행렬을 기술하며, 표적의 속력을 특정한다면 기동하는 표적의 속도 및 궤적을 NED 좌표계에서 나타낼 수 있다. 

---

### Proportional Navigation Guidance

##### 1) 2차원

![proportionalNavigation](https://user-images.githubusercontent.com/70247735/99353931-d79f5800-28e8-11eb-9db1-1e7ccef7077f.png)

비례 항법 유도라고 불리는 Proportional Navigation Guidance의 개념을 설명하기 위한 2차원 그림은 위와 같다. Proportional Navigation Guidance는 위 그림에서 나타나는 *line of sight rate*(&lambda; *dot*)이 0이 되도록 *Kill Vehicle*를 유도하는 방법이며, *Kill Vehicle*과 *target*의 이동 경로를 시각화한 삼각형을 *collison triangle* 이라고 한다.<br>

2차원에서의 *Kill Vehicle*의 가속도는 다음과 같이 나타낼 수 있다.<br>

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?\Large&space;\begin{align*}a_m&=&N\dot{\lambda}V\end{align*}"/>
 
위의 수식에서 알 수 있듯이, *Kill Vehicle*의 가속도 *a<sub>m*은 *Kill Vehicle*의 순간 속도 벡터에 수직이며, 이때 *N*은 무차원의 비례상수, &lambda; *dot*은 *line of sight rate*, *V*는 *closing velocity*이다.<br>

##### 2) 3차원

![p1](https://user-images.githubusercontent.com/70247735/99631405-df910080-2a7e-11eb-8183-e886ea3eaac6.png)

3차원에서의 Proportional Navigation Guidance의 개념을 표현한 그림은 위와 같다. 3차원에서의 *Kill Vehicle* 가속도의 기본형은 다음과 같이 나타낼 수 있다.<br>

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?\Large&space;\begin{align*}a_m&=&NV_r\times\Omega\end{align*}" />
  
위 수식에서 *N*는 무차원의 비례상수이고, *V*는 *Kill Vehicle대한 target 속도*이다. *&Omega; vector*는 *line of sight의 rotation vector*이며, 다음과 같이 나타낼 수 있다.<br>

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?\Large&space;\begin{align*}\Omega&=&\frac{R{\times}V_r}{R{\cdot}R}\end{align*}" />

3차원에서 *Kill Vehicle*의 가속도는 *Kill Vehicle*의 속도 벡터와 *R vector*(range from *Kill Vehicle* to *target*)에 대해 나타낼 수 있으며, 다음과 같이 나타낼 수 있다.<br>

- *R vector*에 수직인 가속도

<img src="https://user-images.githubusercontent.com/70247735/99631389-da33b600-2a7e-11eb-99e1-246a4324c807.png" width="500" height="500"><img src="https://latex.codecogs.com/svg.latex?\Large&space;\begin{align*}a_m&=&-N{\vert}V_r{\vert}\frac{R}{{\vert}R{\vert}}{\times}\Omega\end{align*}" />
  
- *Kill Vehicle* 속도 벡터에 수직인 가속도

 <img src="https://user-images.githubusercontent.com/70247735/99631369-d30ca800-2a7e-11eb-840e-228ef68ca42b.png" width="500" height="500"><img src="https://latex.codecogs.com/svg.latex?\Large&space;\begin{align*}a_m&=&-N{\vert}V_r{\vert}\frac{V_m}{{\vert}V_m{\vert}}{\times}\Omega\end{align*}" />


---

### Kill Vehicle Kinematics and Dynamics

---


### Divert Attitude Control System

![Kill Vehicle Graphics](https://user-images.githubusercontent.com/55905711/99368419-76827f00-28fe-11eb-9936-9bc703d17989.png)

본 연구에서 고려하는 Kill Vehicle은 [RIM-161 Standard Missile 3](https://en.wikipedia.org/wiki/RIM-161_Standard_Missile_3)의 [Kill Vehicle](https://youtu.be/GGvlNufdeL8)과 유사한 형태를 가지고 있다고 가정한다. 동체 전면부에는 표적 포착을 위한 탐색기가 위치하며, 질량중심 주변에 4개의 추력기로 구성된 Divert Control System이, 동체 후면부에 6개의 추력기로 구성된 Attitude Control System이 부착된 형태이다.

![Kill Vehicle Thrusters](https://user-images.githubusercontent.com/55905711/99400646-8adc7100-292a-11eb-82d2-eb7e8344d26e.png)

Kill Vehicle에 부착된 각 추력기들을 위와 같이 나타내었다. 여기서 *D<sub>1</sub>*, *D<sub>2</sub>*, *D<sub>3</sub>*, *D<sub>4</sub>* 와 *A<sub>1</sub>*, *A<sub>2</sub>*, *A<sub>3</sub>*, *A<sub>4</sub>*, *A<sub>5</sub>*, *A<sub>6</sub>* 는 각 DCS 및 ACS 추력기들이 발생시키는 추력을 의미하며, *f<sub>y</sub>*, *f<sub>z</sub>* 는 동체 좌표계의 2축, 3축 방향으로 Kill Vehicle의 질량중심에 작용하는 힘을, 그리고 *l*, *m*, *n* 은 Kill Vehicle의 자세각, 즉 Roll, Pitch, Yaw 를 발생시키기 위한 토크를 나타낸다.

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{align*}f_y&=-D_2+D_4-A_2-A_3+A_5+A_6\\f_z&=D_1-D_3+A_1-A_4\\l&=-bA_2+bA_3-bA_5+bA_6\\m&=aA_1-aA_4\\n&=aA_2+aA_3-aA_5-aA_6\end{align*}" />  

Kill Vehicle에 작용하는 힘과 토크는 다음과 같다. 6개의 ACS 추력기들이 동체 후면부에만 존재하므로, 그 위치는 질량중심에서 크게 벗어난 동시에 질량중심 기준에서 비대칭을 이룬다. 따라서 자세제어를 위해 ACS 추력기를 작동시키는 순간 Kill Vehicle은 토크 *l*, *m*, *n* 뿐만 아니라 힘 *f<sub>y</sub>*, *f<sub>z</sub>* 를 동시에 받을 것이다. *f<sub>y</sub>*, *f<sub>z</sub>* 를 기술하는 식에 ACS 추력기에 의한 *A<sub>1</sub>*, *A<sub>2</sub>*, *A<sub>3</sub>*, *A<sub>4</sub>*, *A<sub>5</sub>*, *A<sub>6</sub>* 항이 포함된 것은 이 때문으로, 동체 전면부에 *f<sub>y</sub>*, *f<sub>z</sub>* 를 상쇄할 수 있는 별도의 추력기가 질량중심 기준 대칭점에 존재하지 않는 한 불가피한 문제이다. *l*, *m*, *n* 을 기술하는 식은 상대적으로 간단하나, ACS 추력기들에 의한 추력을 토크로 변환하는 과정에서 설계변수 *a*, *b*, 즉 질량중심과 ACS 추력기들 사이의 거리 정보가 사용되었다.

상기한 Linear Equation 은 다음과 같은 형태로 정리할 수 있다.

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{align*}b&=Ax\end{align*}"/> 

여기서

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{align*}x&=\begin{bmatrix}D_1&D_2&D_3&D_4&A_1&A_2&A_3&A_4&A_5&A_6\end{bmatrix}^T,\\b&=\begin{bmatrix}f_y&f_z&l&m&n\end{bmatrix}^T,\\A&=\begin{bmatrix}0&-1&0&1&0&-1&-1&0&1&1\\1&0&-1&0&1&0&0&-1&0&0\\0&0&0&0&0&-b&b&0&-b&b\\0&0&0&0&a&0&0&-a&0&0\\0&0&0&0&0&a&a&0&-a&-a\end{bmatrix}\end{align*}"/>  

*x* 는 각 추력기가 발생시키는 추력, *b* 는 Kill Vehicle에 작용하는 힘과 토크, *A* 는 상기한 두 물리량 사이의 관계를 나타내는 행렬이다. 비례항법유도 및 자세제어기에서 연산된 *b* 를 추종하기 위해 DACS 추력기를 어떻게 작동시켜야 하는지, 즉 매 순간 *x* 의 값을 어떻게 계산할지가 우리의 관심사이다.

---

#### Attempt 1 - Least Norm Problem

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{align*}&\underset{x}{\text{minimize}}&&{\lVert}x{\rVert}\\&\text{subject~to}&&b=Ax\end{align*}"/>

우선 위와 같은 형태의 [least norm problem](https://see.stanford.edu/materials/lsoeldsee263/08-min-norm.pdf) 을 고려하자. 이러한 문제는 아래와 같은 해를 가짐이 알려져 있다.

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{align*}x^*=A^\dagger{b}\end{align*}"/>    

여기서

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{align*}A^\dagger=A^T(AA^T)^{-1}\end{align*}"/>  

행렬 *A* 가 [full rank](https://en.wikipedia.org/wiki/Rank_(linear_algebra))이고, [underdetermined](https://en.wikipedia.org/wiki/Underdetermined_system)한 형태이므로 *A* 의 [Moore-Penrose pseudoinverse matrix](https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse)는 위와 같다. Pseudoinverse matrix를 이용한 해는 DACS 추력기를 최소한으로 사용하도록 작동하지만, 물리적으로 구현이 불가능하다. 해당 해는 실수 전체의 범위를 가지는 반면, 현실의 추력기들은 한쪽 방향으로만 추력을 발생시킬 수 있으므로 추력값의 범위가 음이 아닌 값으로 제한되기 때문이다.

---

#### Attempt 2 - Projected Gradient Descent

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{align*}&\underset{x}{\text{minimize}}&&{\lVert}Ax-b{\rVert}_2^2\\&\text{subject~to}&&x\geq0\end{align*}"/>

기존 least norm problem 의 [cost function](https://en.wikipedia.org/wiki/Loss_function)을 다음과 같은 [least squared error](https://en.wikipedia.org/wiki/Mean_squared_error) 형태로 수정하는 한편, inequality constraint를 추가하였다. 이 문제의 해결을 위해 [gradient descent](https://en.wikipedia.org/wiki/Gradient_descent) 알고리즘을 고려하자. Gradient descent 는 cost function의 local minimum을 찾는 알고리즘으로, 매 step을 반복하며 cost function의 gradient가 0으로 수렴하는 벡터 *x* 를 찾는 것이 목적이다.

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{align*}&x_{temp}=x_k-h_k\nabla_xf(x_k),\\&\text{if~}f(x_{temp})\leq{f(x_k)}\\&~~~~~~x_{k+1}=x_{temp},~h_{k+1}=1.2h_k\\&\text{else}\\&~~~~~~x_{k+1}=x_{k},~h_{k+1}=0.5h_k\\\end{align*}"/>

여기서 *h* 는 learning rate, *f(x)* 는 cost function이며, *x* 에 대한 *f(x)* 의 gradient는 *x* 의 변화에 따른 함수 *f(x)* 의 최대 변화율을 나타낸다.

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{align*}f(x)={\lVert}Ax-b{\rVert}_2^2,~\nabla_xf(x)=2A^T(Ax-b)\end{align*}"/>

먼저 inequality constraint가 고려되지 않은 경우를 가정하자. 벡터 *x* 는 *f(x)* 의 local minimum을 지나치기 전까지 learning rate를 120%씩 증가시키며 하강한다. Local minimum을 지나치면 learning rate를 50%씩 감소시키며 local minimum에 접근한다. Iteration을 충분히 반복했거나 cost function의 gradient가 충분히 0에 가까워졌다고 판단되면 알고리즘을 종료한다.

![Gradient Descent](https://user-images.githubusercontent.com/55905711/99520611-a43af700-29d6-11eb-99a8-950ba4967123.png)

시각화를 위해 *x* 를 2차원 벡터로 가정한 뒤 임의의 cost function에 대해 gradient descent를 적용하였다. 몇 번의 iteration 후 성공적으로 *x* 가 local minimum으로 수렴하는 것을 확인할 수 있다.

이제 inequality constraint를 고려할 차례이다. 위 그림의 붉은 영역은 벡터 *x* 가 존재하면 안 되는 영역, 즉 constraint를 나타낸다. 우리의 feasible set *C* 는 그림 우측 상단의 nonnegative region으로 제한되므로, 매 step에서 구한 *x* 가 *C* 를 벗어나면 *x* 를 *C* 에 대해 projection해 constraint를 충족시킬 것이다.

![Projected Gradient Descent](https://user-images.githubusercontent.com/55905711/99521404-b8cbbf00-29d7-11eb-8cf3-13ab5009eaf5.png)

동일한 예시에 projected gradient descent를 적용해 보았다. Inequality constraint를 만족하는 *x* 가 적절한 최솟값으로 수렴해 나가는 것을 확인할 수 있다. 마찬가지로, 추력기가 발생시킬 수 있는 최대 추력이 제한된 경우에도 본 알고리즘을 적용해 적절한 최적해를 구할 수 있다.

---

#### Attempt 3 - Projected Gradient Descent with Regularizer

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{align*}&\underset{x}{\text{minimize}}&&{\lVert}Ax-b{\rVert}_2^2+\lambda{\lVert}x{\rVert}_2^2+\gamma{\lVert}x-x_{prev}{\rVert}_2^2\\&\text{subject~to}&&x\geq0\end{align*}"/>

Least squared error의 형태를 가진 기존 cost function을 확장해 다음과 같은 cost function을 새롭게 정의하였다. 첫 번째 regularizer는 추력 자체의 최소화를, 두 번째 regularizer는 이전 timestep에서 계산된 추력과 현재 계산된 추력의 차이를 최소화하도록, 즉 추력을 시간에 대해 연속적으로 발생시킬 것을 의미하며, 비례상수 *&lambda;* 와 *&gamma;* 는 각 regularizer들의 가중치에 해당한다.

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{align*}f(x)={\lVert}Ax-b{\rVert}_2^2+\lambda{\lVert}x{\rVert}_2^2+\gamma{\lVert}x-x_{prev}{\rVert}_2^2,~\nabla_xf(x)=2A^T(Ax-b)+2\lambda{x}+2\gamma(x-x_{prev})\end{align*}"/>

새롭게 정의된 cost function의 gradient는 다음과 같다. 동일한 방법으로 projected gradient descent 알고리즘을 적용해 적절한 추력벡터 *x* 를 구할 수 있지만, 시뮬레이션 모듈에 이를 적용할 경우 실행시간이 상당히 길어지는 문제가 발생했다. 프로그램을 최적화함으로서 실행시간을 어느 정도 단축시킬 수 있지만, 여기서는 추력값 문제 자체를 단순화시키는 방안을 모색해 보았다.

#### Attempt 4 - Paired Thrusters

![Paired Thrusters](https://user-images.githubusercontent.com/55905711/99669928-184acd00-2ab3-11eb-93db-b488218f8345.png)

음수 추력을 고려하기 위해 10개의 DACS 추력기들을 두 개씩 짝지어 새롭게 정의하자. 서로 대칭인 추력기들을 묶으면 기존 [*D<sub>1</sub>*, *D<sub>3</sub>*] 를 *D&prime;<sub>1</sub>* 로, [*D<sub>2</sub>*, *D<sub>4</sub>*] 를 *D&prime;<sub>2</sub>* 로, [*A<sub>1</sub>*, *A<sub>4</sub>*] 를 *A&prime;<sub>1</sub>* 로, [*A<sub>2</sub>*, *A<sub>6</sub>*] 를 *A&prime;<sub>2</sub>* 로, [*A<sub>3</sub>*, *A<sub>5</sub>*] 를 *A&prime;<sub>3</sub>* 라고 생각할 수 있다. 새롭게 정의된 5개의 추력기들이 동체 좌표계와 동일한 방향으로 추력을 발생시킨다고 가정하자. 예를 들어 *D&prime;<sub>1</sub>* 추력기가 양의 추력을 발생시키면 기존 *D<sub>1</sub>* 추력기가 작동해 Kill Vehicle은 Yaw 축 방향으로 힘을 받고, 음의 추력을 발생시키면 *D<sub>3</sub>* 추력기가 작동하여 Yaw 축의 반대 방향으로 힘을 받을 것이다. 

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{align*}f_y&=D_2^{'}+A_2^{'}+A_3^{'}\\f_z&=D_1^{'}+A_1^{'}\\l&=bA_2^{'}-bA_3^{'}\\m&=aA_1^{'}\\n&=-aA_2^{'}-aA_3^{'}\end{align*}" />  

이전과 마찬가지로 새롭게 정의한 추력기들을 이용해 Kill Vehicle에 작용하는 힘과 토크를 나타내었다. 이 Linear Equation을 간단하게 정리하면 다음과 같다.

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{align*}b&=A^{'}x^{'}\end{align*}"/> 

여기서

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{align*}x^{'}&=\begin{bmatrix}D_1^{'}&D_2^{'}&A_1^{'}&A_2^{'}&A_3^{'}\end{bmatrix}^T,\\b&=\begin{bmatrix}f_y&f_z&l&m&n\end{bmatrix}^T,\\A^{'}&=\begin{bmatrix}0&1&0&1&1\\1&0&1&0&0\\0&0&0&b&-b\\0&0&a&0&0\\0&0&0&-a&-a\end{bmatrix}\end{align*}"/>  

매우 편리하게도 추력벡터 *x&prime;* 는 힘과 토크 벡터 *b* 와 같은 크기를 가지며, *A&prime;* 또한 nonsingular matrix이다. 따라서 *x&prime;* 는 아래와 같은 해를 가짐을 쉽게 유추할 수 있다.

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{align*}x^{'}={A^{'}}^{-1}b\end{align*}"/> 

이렇게 구한 추력벡터 *x&prime;* 는 기존 방법으로 구한 해와 매우 유사한 결과를 보여주며, 그 실행시간 역시 현저히 짧다. 하지만 2개의 추력기들을 하나로 묶어 고려하는 특성 상, 모든 추력기들을 개별적으로 제어하는 것이 불가능하다는 한계 역시 가지고 있다.

---

### detect target

Kill Vehicle에는 seeker가 달려있다. 이 target을 detect 하는 것을 그림으로 나타내면 아래와 같다.<br>

![sensor1](https://user-images.githubusercontent.com/70247735/99638342-6a76f880-2a89-11eb-93ff-5be779cf730a.png)

위 그림을 보기 편하게 2차원으로 나타내면 다음과 같이 나타낼 수 있다.<br>

![detect](https://user-images.githubusercontent.com/70247735/99893521-4a7f4900-2cc4-11eb-88b8-ebd401336f5c.png)

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\alpha>\beta:detect"/>
<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\alpha<\beta:non-detect"/>

위의 *&alpha;* 는  Kill Vehicle seeker의 탐색 범위를 나타내는 각이다. 또 *&beta;* 는 Body1 axis를 NED좌표계로 나타낸 축과 LOS를 NED좌표계로 나타낸 축 사이의 각이다. 이때 *&alpha;* 가 *&beta;* 보다 크면 Kill Vehicle seeker가 Target을 탐지하는 것이고, *&alpha;* 가 *&beta;* 보다 작으면 Kill Vehicle seeker가 Target을 탐지하지 못하는 것이다.<br>
Body1 axis를 NED좌표계로 나타낸 축과 LOS를 NED좌표계로 나타낸 축은 아래와 같다.

Body 1 axis to NED
<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{bmatrix}Body\\to\\NED\end{bmatrix}\begin{bmatrix}1\\0\\0\end{bmatrix}"/>
 
Los to NED
<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;\begin{bmatrix}Los\\to\\NED\end{bmatrix}\begin{bmatrix}1\\0\\0\end{bmatrix}"/>
 
마지막으로 여러 좌표계에 대한 rotation 행렬은 다음과 같이 나타낼 수 있다.

1. NED to Body
<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;N2B=\begin{bmatrix}1&0&0\\0&\cos\phi&\sin\phi\\0&-\sin\phi&\cos\phi\end{bmatrix}\begin{bmatrix}\cos\theta&0&-\sin\theta\\0&1&0\\\sin\theta&0&\cos\theta\end{bmatrix}\begin{bmatrix}\cos\psi&\sin\psi&0\\-\sin\psi&\cos\psi&0\\0&0&1\end{bmatrix}"/><br>

2. Body to NED
<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;B2N=\begin{bmatrix}N2B\end{bmatrix}^{-1}=\begin{bmatrix}N2B\end{bmatrix}^\intercal"/>

3. NED to Wind
<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;N2W=\begin{bmatrix}\cos(FPA)&0&-\sin(FPA)\\0&1&0\\\sin(FPA)&0&\cos(FPA)\end{bmatrix}\begin{bmatrix}\cos(HED)&\sin(HED)&0\\-\sin(HED)&\cos(HED)&0\\0&0&1\end{bmatrix}"/>
 
4. Wind to NED
<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;W2N=\begin{bmatrix}N2W\end{bmatrix}^{-1}=\begin{bmatrix}N2W\end{bmatrix}^\intercal"/>

5. Wind to Body
<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;W2B=\begin{bmatrix}N2B\end{bmatrix}\begin{bmatrix}W2N\end{bmatrix}"/>
 
6. Body to Wind
<p align="center">
<img src="https://latex.codecogs.com/svg.latex?&space;B2W=\begin{bmatrix}W2B\end{bmatrix}^{-1}=\begin{bmatrix}W2B\end{bmatrix}^\intercal"/>


---
