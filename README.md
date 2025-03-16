# 双平台空对空目标定位与跟踪仿真环境

![gazebo_sideview](/gazebo_sideview.png)

![simulation_rviz](/simulation_rviz.png)

## Usage

```bash
cd your_workspace/src
git clone https://github.com/fbh16/npurobocourse_sim.git
catkin_make

#Terminal1:启动仿真环境
source devel/setup.bash
roslaunch hector_quadrotor_demo double_drones_three_targets.launch

#Terminal2:观测机1,并按‘0’起飞
roslaunch observe_controller obs01_control.launch
#Terminal3:观测机2,并按‘0’起飞
roslaunch observe_controller obs02_control.launch

#Terminal4:目标机1，并按‘0’进行轨迹跟踪
roslaunch target_drone_controller target_01_controller.launch
#Terminal5:目标机1，并按‘0’进行轨迹跟踪
roslaunch target_drone_controller target_02_controller.launch
#Terminal6:目标机1，并按‘0’进行轨迹跟踪
roslaunch target_drone_controller target_03_controller.launch
```



## Copyright

Please check [here](LICENSE.txt).
