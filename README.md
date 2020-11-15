# cooperative_guidance
Cooperative guidance for multiple kill vehicles

code : MATLAB & python
## Simulator with Single Target and Kill Vehicle
본 시뮬레이션 모듈은 단일 표적 격추를 위해 기동하는 한 대의 Kill Vehicle을 모사한다.
### Simulator Structure
![Simulator Structure](https://user-images.githubusercontent.com/55905711/99185203-ba08ac00-278b-11eb-850d-b8e440bdb006.png)
사용자는 시뮬레이션 시작 전 표적 및 Kill Vehicle의 초기 위치, 진행방향, 속력 등을 입력한다. 시뮬레이션 모듈은 입력받은 초기 파라미터를 기반으로 표적 궤적을 적분하며, Kill Vehicle은 생성된 표적 궤적을 기반으로 [비례항법유도(Proportional Navigation Guidance)](https://en.wikipedia.org/wiki/Proportional_navigation)를 실시, 표적을 향해 기동한다. Kill Vehicle과 표적 간의 상대속도가 양수에서 음수로 변하면 표적이 격추되었다고 가정, 시뮬레이션을 종료하고 궤적 및 속도 데이터를 출력한다. 
### Target Trajectory

### Kill Vehicle Kinematics and Dynamics
