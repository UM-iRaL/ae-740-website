---
layout: default
title: "Exercises"
parent: "Lab 4"
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
Each group should create a new folder called `TEAM_<N>` replacing `<N>` with your team number. For example team 2 will name their folder `TEAM_2`. Please put the source code of the entire `controller_pkg` package in the folder `TEAM_2`. Zip the folder and submit to Canvas.

Each team will only need to submit one `TEAM_<N>.zip` to Canvas.

**Deadline:** To submit your solution, please upload the corresponding files under `Assignment > Lab 4` by **Mon, Oct 13 11:59 EST**.

## Overview
In Lab 4, you will extend the MPC controller from Lab 3 to implement **target tracking** capabilities. The main difference is that instead of following predefined trajectories, your drone will obtain the position of a target drone in real-time and use the Model Predictive Control (MPC) framework developed in Lab 3 to track the target. We launch two drones in the simulator: `cf_1` (the target) and `cf_2` (the pursuer). Each drone runs its own instance of the MPC controller. The target drones runs the MPC controller from Lab 3 to track fixed trajectories, whereas `cf_2` receives real-time position updates of `cf_1` via a ROS2 topic subscription. For this lab, the deliverable includes the new MPC controller node, that tracks a specific target drone. We will use the same simulator for the Crazyflie quadrotor, and also the same `controller_pkg` ROS2 package that we developed in Lab 3. The new files for Lab 4 are provided in the `lab4` folder of the `ae740_labs` repository. 


## Instructions for using the files 
1. Pull the latest version of `ae740_labs` repository to get the Lab 4 files (is the `lab4` folder):

2. Copy the `target_tracking_mpc.py` file from `lab4` folder to your existing `ros2_ws/src/controller_pkg/controller_pkg/` directory, that we used in Lab 3. This is important as this file is the equivalent for the `crazyflie_mpc.py` file and also calls the `quadrotor_simplified_model.py` and `tracking_mpc.py` files that you developed in Lab 3.    

3. Inside the `ros2_ws/src/controller_pkg` directory, create a new folder called `launch` and add a new launch file `launch_lab4_target_tracking.launch..py` provided in the `lab4` folder. This launch file will launch two drones -- one with the `target_tracking_mpc.py` node, and the other with the `crazyflie_mpc.py` node.

4. **Replace** the existing `setup.py` file in `controller_pkg` with the one provided in the `lab4` folder. This is important as it ensures that the new launch file can be found and executed correctly.

## Instruction for running the target tracking simulation
*These steps assume you completed the coding part in the `target_tracking_mpc.py` file.*

**Terminal 1:** Launch Gazebo SITL with 2 drones (inside `ae740_crazyflie_sim` directory)
```bash
bash crazyflie_firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_target_tracking.sh -t 1 -p 1 -m crazyflie
```

**Terminal 2:** Luanch the Crazyswarm2 server                                          
```bash
cd ros2_ws/ 
colcon build --symlink-install  # if not already built
source install/setup.bash
ros2 launch crazyflie launch.py backend:=cflib
```

**Terminal 3:** Launch MPC controllers for both drones
```bash
cd ros2_ws/ && source install/setup.bash 
ros2 launch controller_pkg launch_lab4_target_tracking.py
```

**Terminal 4:** Commands to takeoff and start the target tracking
```bash
# Step 1: Takeoff both drones
ros2 topic pub -t 1 /all/mpc_takeoff std_msgs/msg/Empty

# Step 2: Start the tracking simulation
ros2 topic pub -t 1 /all/mpc_trajectory std_msgs/msg/Empty
```
**Note:** Sometimes, rarely though, you might get the warning `Empty state message.`, which is because either position or velocity of one of the drones is not properly initialized on the ROS2 server. To resolve, re-run the main commands on terminals 1 and 2 before attempting again.

## Deliverable 1 - Complete the Target Tracking MPC Implementation (80 pts)
The new controller node `target_tracking_mpc.py` has most of the structure same as `crazyflie_mpc.py` node used in Lab 3. The key differences are summarized below:

### Lab 3 (Basic MPC)
- The main controller node: `crazyflie_mpc.py`
- Single drone following predefined trajectories 
- Static reference generation
- No real-time target information

### Lab 4 (Target Tracking MPC)
- The main controller node: `target_tracking_mpc.py`
- **Two drones**: `cf_1` (target) and `cf_2` (pursuer/tracker)
- **Real-time target tracking**: `cf_2` follows `cf_1`'s position
- **Dynamic reference generation** based on target drone's pose
- **New trajectory type**: `'target_tracking'` 
- **Additional subscriber**: Target drone position subscriber

In the `target_tracking_mpc.py` file, the comments `# [TODO LAB 4]` indicate where you need to implement the additions for the target tracking functionality, in contrast to the `crazyflie_mpc.py` file from Lab 3. You can specifically search for `[TODO LAB 4]` to find the changes. The starter code include the `TODO` comments from Lab 3 as well, which you can ignore after finishing the Lab3 exercises.


**Similarities with Lab 3:**
- Position subscriber for own drone (`cf_2`)
- Velocity subscriber for own drone (`cf_2`) 
- MPC solution path publisher
- Attitude setpoint command publisher
- Similar structure for initialization, timers, and MPC solver integration
- Target (`cf_1`) runs using the basic MPC controller from Lab 3 

**Additions for Lab 4:**
- New variable `target_name` is introduced to specify the target drone.
- The target tracking functionality is added by introducing a trajectory type `'target_tracking'` in the `trajectory_function()`, where the reference position is set to the latest received target drone position.
- A new subscriber and callback function is created to listen to the target drone's position.



## Deliverable 2 - Testing and Validation (20 pts)
Add a new trajectory with name `lemniscate` in the `trajectory_function()` of `crazyflie_mpc.py` file. The lemniscate (figure-eight) trajectory can be defined as:

**Parameters:**
- `a = 1.0` 
- `b = 0.5 * tanh(0.1 * t)` (for smooth start)

**Position:**
```
x_ref(t) = x_start + a * sin(b * t)
y_ref(t) = y_start + a * sin(b * t) * cos(b * t)  
z_ref(t) = z_start
```

**Velocity:**
```
vx_ref(t) = a * b * cos(b * t)
vy_ref(t) = a * b * cos(2 * b * t)
vz_ref(t) = 0.0
```

## Expected Behavior
- `cf_1` should follow its predefined trajectory (circle, lemniscate, etc.)
- `cf_2` should track and follow `cf_1`'s position
- Both drones should maintain stable flight
- The tracking should be smooth without oscillations

<p align="center">
    <img src="../../../assets/img/lab4/target_tracking.png" alt="Target Tracking Example" style="width:90%;">
</p>

## Deliverables:
1. **Working code** (`target_tracking_mpc.py` file only) that compiles and runs without errors
2. **Video demonstration** (30-60 seconds) showing:
   - Both drones taking off
   - `cf_1` following a trajectory  
   - `cf_2` successfully tracking `cf_1`
   - Two separate videos for `circle` and `lemniscate` trajectories
3. **Parameter tuning** documentation explaining any changes made to MPC parameters (changes to number of steps, weights, control rate can be needed for stable tracking)



