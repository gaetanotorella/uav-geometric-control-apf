# Geometric control and artificial potential fields for a quadrotor UAV

<p align="center"><img src="docs/media/simulink_trajectory.png" width="420" alt="Quadrotor trajectory around two cylindrical obstacles in MATLAB/Simulink"></p>

A quadrotor that flies to a goal while avoiding obstacles: a **geometric controller on SO(3)** tracks the
reference produced by an **artificial potential field (APF)**, and a **momentum-based estimator** of the external
wrench corrects the mass used by the controller. The method is validated in MATLAB/Simulink and then in
ROS/Gazebo with the [RotorS](https://github.com/ethz-asl/rotors_simulator) simulator, where the APF also runs online
on the point cloud of a LiDAR.

Final project of the **Field and Service Robotics** course (Prof. Fabio Ruggiero), M.Sc. in Automation Engineering
and Robotics, University of Naples Federico II, academic year 2023/24.
Authors: **Stefano Riccardi** and **Gaetano Torella**. Full report: [docs/report.pdf](docs/report.pdf).

## Method

<div align="center">

| <img src="docs/media/scheme_apf.png" width="400" alt="Block diagram of the whole system"> | <img src="docs/media/scheme_geometric_control.png" width="320" alt="Block diagram of the geometric controller"> |
|:---:|:---:|
| The whole system: the APF moves the reference of the controller | The geometric controller: trajectory and attitude tracking |

</div>

- **Geometric control.** An outer loop turns the position error into the desired thrust and attitude; an inner
  loop tracks the attitude on SO(3) and computes the torques. Working on rotation matrices avoids the
  singularities of Euler angles.
- **Artificial potential field.** The goal attracts the UAV and every obstacle repels it within an influence
  distance; the total force is used as the velocity that moves the reference. The obstacles are discretised into
  points on their surface. Three versions:
  - *waypoints*: the APF is integrated offline into timed waypoints, and a minimum-jerk trajectory through them is
    the reference of the controller;
  - *in the loop*: the APF is evaluated at every step of the simulation from the current position. It gives
    similar results, but it is the version that can run online, so it is the one ported to ROS;
  - *online* (ROS only): the repulsive force is computed from the LiDAR points closer than the influence distance,
    so the obstacles do not need to be known in advance.
- **Momentum-based estimator.** An estimator of order r (gains from a Butterworth filter) reconstructs the external
  force and torque. In this project it is used to estimate the real mass of the UAV.

## Results

**MATLAB/Simulink.** Quadrotor of 1.52 kg from (0, 0, 0) to (10, 10, 10) m around two cylindrical obstacles. With
the APF in the loop, the position error of the geometric controller stays around a millimetre:

<p align="center"><img src="docs/media/simulink_position_error.png" width="640" alt="Position error in Simulink"></p>

**ROS/Gazebo (RotorS, AR.Drone model).** The controller sends roll, pitch, yaw rate and thrust commands to RotorS.

*Mass estimation.* With the nominal mass of 1.5 kg the position error keeps an offset of about 0.2 m along z. The
estimator gives a real mass of about **1.63 kg**; with this value the steady-state error goes to zero:

<div align="center">

| <img src="docs/media/position_error_mass_1.5.png" width="372" alt="Position error with a mass of 1.5 kg"> | <img src="docs/media/position_error_mass_1.6335.png" width="348" alt="Position error with a mass of 1.6335 kg"> |
|:---:|:---:|
| m = 1.5 kg | m = 1.6335 kg (note the different scales) |

</div>

*Online APF.* The UAV avoids the obstacles it sees with the LiDAR, without knowing them in advance:

<p align="center"><img src="docs/media/lidar_point_cloud.png" width="420" alt="The UAV and the LiDAR point cloud of an obstacle in RViz"></p>

As with every potential field, it can stop in a **local minimum**. In the `test_city` world the last goal is beyond
an obstacle: the attractive and repulsive forces almost cancel out and the UAV does not reach it.

<div align="center">

| <img src="docs/media/test_city_world.jpg" width="356" alt="The test_city world in Gazebo"> | <img src="docs/media/local_minimum_rviz.png" width="363" alt="The local minimum in RViz"> |
|:---:|:---:|
| The `test_city` world in Gazebo | The local minimum in RViz |

</div>

Offline APF in Gazebo:

https://github.com/user-attachments/assets/55e5460c-3663-461d-9f11-0e77646f7d14

Online APF with the LiDAR (Gazebo on top, onboard camera bottom left, point cloud in RViz bottom right):

https://github.com/user-attachments/assets/e65f0698-3dd9-4095-9674-cb3982ff717d


## Repository structure

```
├── matlab/
│   ├── apf_waypoint_offline/   Simulink: APF waypoints + minimum-jerk trajectory
│   ├── apf_offline/            Simulink: APF evaluated in the loop
│   ├── estimator/              momentum-based estimator on a ROS bag (mass estimation)
│   └── rosbag_plots/           plots of the ROS/Gazebo experiments from a ROS bag
├── ros/
│   ├── geometric_controller/   ROS package: controller, APF (offline/online), estimator, UAV model, worlds
│   ├── xbox_controller/        gamepad teleoperation (optional)
│   └── mav_comm, octomap_ros, rotors_simulator   (git submodules)
└── docs/                       report and images
```

## MATLAB/Simulink

Requirements: MATLAB and Simulink with UAV Toolbox, Robotics System Toolbox and ROS Toolbox.

Open MATLAB in one of the folders of `matlab/` and run its script: `apf_waypoint_offline_script.m`,
`apf_offline_script.m`, `estimator_script.m` or `plot_rosbag.m`. The last two read a ROS bag recorded in Gazebo:
`UAV.launch` (see below) records it in `~/geom_controller.bag`; copy it into the folder of the script.

## ROS/Gazebo

Tested on Ubuntu 20.04 with ROS Noetic and `catkin_tools`.

```bash
cd ~/catkin_ws/src
git clone --recursive https://github.com/gaetanotorella/uav-geometric-control-apf.git
cd ~/catkin_ws && catkin build && source devel/setup.bash
```

1. Gazebo and the UAV (default world: `APF_world`, the others are in `ros/geometric_controller/worlds`):
   ```bash
   roslaunch geometric_controller UAV.launch world_name:=APF_world
   ```
   This launch file also records the main topics in `~/geom_controller.bag` (arguments `bag_directory` and
   `bag_name`).
2. The geometric controller:
   ```bash
   roslaunch geometric_controller geometric_controller_apf.launch
   ```
3. The APF and the estimator, with the goal position:
   ```bash
   roslaunch geometric_controller apf_method.launch version:=offline x_goal:=0 y_goal:=0 z_goal:=2
   ```
   `version:=offline` uses the known obstacles, `version:=online` the LiDAR.

## References

- F. Furrer, M. Burri, M. Achtelik, R. Siegwart, *RotorS—A Modular Gazebo MAV Simulator Framework*, in Robot
  Operating System (ROS): The Complete Reference (Volume 1), Springer, 2016.
- A. Ma'Arif et al., *Artificial potential field algorithm for obstacle avoidance in UAV quadrotor for dynamic
  environment*, IEEE COMNETSAT, 2021.
- The full list is in the [report](docs/report.pdf).
