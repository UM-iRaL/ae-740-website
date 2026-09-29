---
layout: default
title: "Exercises"
parent: "Lab 3"
grand_parent: Labs
nav_order: 1
has_toc: True
---


# Exercises
{: .no_toc .text-delta .fs-9 }


1. TOC
{:toc}


## Submission

### Team

Each group should create a new folder called `TEAM_<N>` replacing `<N>` with your team number. For example team 2 will name their folder `TEAM_2`. Zip the folder and submit to Canvas. Each team will only need to submit one `TEAM_<N>.zip` to Canvas.

The folder must contain:
1. **Report:** a brief report (PDF format) with your team member names and IDs, the controller gains you used and how you tuned them, a short discussion of the tracking performance for each trajectory, and the names of the video files.
2. **Code:** the source code of the entire `controller_pkg` package.
3. **Videos:** the demo videos of Deliverables 2 and 3. Keep each video short (under 1 minute) and compressed (e.g., MP4 at 720p), so that the zip file stays under 100 MB.

**Deadline:** To submit your solution, please upload the corresponding files under `Assignment > Lab 3` by **Wed, Oct 7 11:59 EST**.


## Trajectory tracking for UAVs
In this lab, we are going to implement the geometric controller on the Crazyflie software-in-the-loop (SITL) simulator based on Gazebo. This also enables us to deploy our controller on the hardware experiments.

### Getting the code
The two main repositories we use are -- [ae740_crazyflie_sim](https://github.com/UM-iRaL/ae740_crazyflie_sim) and [ae740_labs](https://github.com/UM-iRaL/ae740_labs)
Follow these steps to set up your environment and get the required packages:

1. First, we download the main simulator for the Crazyflie. Clone the repository (assuming your home directory):
    ```bash
    cd ~
    git clone https://github.com/UM-iRaL/ae740_crazyflie_sim.git
    ```

2. We then pull the latest version of your lab repository, that contains the controller package. The directory `lab3/controller_pkg/controller_pkg` contains the file that you need to edit. Pull the latest version using:
    ```bash
    cd ~/ae740_labs
    git pull
    ```

3. Copy the controller package to the simulator ROS2 workspace:
    ```bash
    cp -r ~/ae740_labs/lab3/controller_pkg ~/ae740_crazyflie_sim/ros2_ws/src/
    ```
    Now that the simulator has been cloned and your file structure has been setup, you can proceed with the build and installation.


### Installation
A detailed installation procedure has been given [**HERE**](https://github.com/UM-iRaL/ae740_crazyflie_sim?tab=readme-ov-file#installation) in the README file of the repo. The basic process includes the following steps --
1. [Dependencies](https://github.com/UM-iRaL/ae740_crazyflie_sim?tab=readme-ov-file#2-system-dependencies), which includes
    *(a)* ROS2 Humble and Gazebo Harmonic
    *(b)* System-wide dependencies (using `sudo apt install` command)
2. [Python Virtual Environment Setup](https://github.com/UM-iRaL/ae740_crazyflie_sim?tab=readme-ov-file#3-python-virtual-environment), including the Python dependencies (inside the `ae740_venv`)
3. [crazyflie-firmware](https://github.com/UM-iRaL/ae740_crazyflie_sim?tab=readme-ov-file#4-crazyflie-firmware-sitl-and-gazebo-plugins) build using cmake
4. [crazyflie-lib-python](https://github.com/UM-iRaL/ae740_crazyflie_sim?tab=readme-ov-file#5-crazyflie-python-library) installation inside `ae740_venv`
5. [Building ROS2 Workspace](https://github.com/UM-iRaL/ae740_crazyflie_sim?tab=readme-ov-file#7-ros-2-workspace) for the Crazyswarm2 ROS2 server and controller package.

The Acados installation in the README is only needed for the MPC controllers, and you can skip it for this lab. We will install it in Lab 5.

Please follow each step carefully, as the setup process is extensive and essential for successful development. This environment is specifically designed to support controller development for hardware experiments. Later in the course, you will use the same codebase to deploy and test your control algorithms directly on the Crazyflie drone hardware!

You can test the Gazebo SITL firmware and `crazyflie_server` modules using following commands from `ae740_crazyflie_sim` directory.

**Terminal 1:**
```
bash crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh
```

**Terminal 2:**
```
cd ros2_ws
source install/setup.bash
ros2 launch crazyflie launch.py backend:=cflib
```

You should see two windows (Gazebo and RViz2) similar to as shown here.

<p align="center">
    <img src="../../../assets/img/lab3/gazebo_and_rviz.png" alt="Gazebo-RViz" style="width:100%;">
</p>

Refer to the installation page in case of errors, as some of the common errors are mentioned [***here***](https://github.com/UM-iRaL/ae740_crazyflie_sim?tab=readme-ov-file#terminal-1-gazebo-sitl). Feel free to post the questions on Piazza if you encounter any errors during the installation.


### Deliverable 1 - Geometric Controller

Your main task is to complete the geometric controller discussed in class (see also the referenced paper [^1]) in the provided template `crazyflie_geometric_controller.py`, in `ae740_crazyflie_sim/ros2_ws/src/controller_pkg/controller_pkg`. This ROS2 node receives the state of the Crazyflie, computes the reference trajectory, and sends the collective thrust and body torque commands to the Crazyflie at a fixed rate.

Complete all the `[TODO]` sections in the file. Once your code is functional, build the workspace and run the controller node (see the [README](https://github.com/UM-iRaL/ae740_crazyflie_sim?tab=readme-ov-file#terminal-3-controller) for the complete list of commands):
```bash
cd ~/ae740_crazyflie_sim/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run controller_pkg crazyflie_geometric_controller
```

Then, use a separate terminal to command the take-off, trajectory, hover and landing:
```bash
ros2 topic pub -t 1 /all/geo_takeoff std_msgs/msg/Empty
ros2 topic pub -t 1 /all/geo_trajectory std_msgs/msg/Empty
ros2 topic pub -t 1 /all/geo_hover std_msgs/msg/Empty
ros2 topic pub -t 1 /all/geo_land std_msgs/msg/Empty
```


### Deliverable 2 - Circular Trajectory

Demonstrate your geometric controller tracking the `horizontal_circle` trajectory. The video must show the Crazyflie taking off, tracking the circle, and landing. In the report, very briefly discuss the tracking performance.


### Deliverable 3 - Trajectory of Your Choice

Add a new trajectory of your choice in `trajectory_function()`, and select it with `self.trajectory_type`. Demonstrate your geometric controller tracking it with a video, and in the report, give the equations of the trajectory and briefly discuss the tracking performance.


Please reach out during office hours or post on Piazza for any bugs, errors or code-related questions!


# References
[^1]: Lee, Taeyoung, Melvin Leoky, N. Harris McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." Decision and Control (CDC), 49th IEEE Conference on. IEEE, 2010 [Link](http://math.ucsd.edu/~mleok/pdf/LeLeMc2010_quadrotor.pdf)
