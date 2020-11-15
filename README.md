# cooperative_guidance
Cooperative guidance for multiple kill vehicles

code : MATLAB & python
## Simulator with Single Target and Kill Vehicle
본 시뮬레이션 모듈은 단일 표적 격추를 위해 기동하는 한 대의 Kill Vehicle을 모사한다. 표적 및 Kill Vehicle의 운동을 모사하기 위해 각 비행체에게 가해지는 모멘트 및 회전 운동을 직접 고려하는 6DOF 모델을 적용하는 것이 이상적이나, 이는 필요 이상으로 과도한 수준의 충실도를 갖는다고 판단된다. 본 시뮬레이션 모듈에는 비행체의 질점을 모사하는 3DOF 모델을 기반으로, Kill Vehicle의 자세각 변화를 추가적으로 모사하는 Pseudo 6DOF 모델을 적용하였다.
### Simulator Structure
![Simulator Structure](https://user-images.githubusercontent.com/55905711/99185203-ba08ac00-278b-11eb-850d-b8e440bdb006.png)
사용자는 시뮬레이션 시작 전 표적 및 Kill Vehicle의 초기 위치, 진행방향, 속력 등을 입력한다. 시뮬레이션 모듈은 입력받은 초기 파라미터를 기반으로 표적 궤적을 적분하며, Kill Vehicle은 생성된 표적 궤적을 기반으로 [비례항법유도(Proportional Navigation Guidance)](https://en.wikipedia.org/wiki/Proportional_navigation)를 실시, 표적을 향해 기동한다. Kill Vehicle과 표적 간의 상대속도가 양수에서 음수로 변하면 표적이 격추되었다고 가정, 시뮬레이션을 종료하고 궤적 및 속도 데이터를 출력한다. 
### Target Trajectory

### Kill Vehicle Kinematics and Dynamics
