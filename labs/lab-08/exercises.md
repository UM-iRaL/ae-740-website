---
layout: default
title: "Exercises"
parent: "Lab 8"
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
Each group should create a new folder called `TEAM_<N>` replacing `<N>` with your team number. For example team 2 will name their folder `TEAM_2`. Zip the folder and submit to Canvas.

Each team will only need to submit one `TEAM_<N>.zip` to Canvas.

For organizing the submission better, please include a single report (PDF format) inside your folder that includes your answers, explanations, video file names and **copy of the Python code** that you edit/write. Please make sure to include your team member names and IDs in the report. 

**Deadline:** To submit your solution, please upload the corresponding files under `Assignment > Lab 8` by **Tue, Nov 11 11:59 EST**.


## Overview
In this lab, we will implement online learning algorithm for wind distrubances and model inaccuracies using the MPC framework. Particularly, the goal is to implement a recent algorithm "Simulataneous System Identification and Model Predictive Control" (SSI-MPC) based on the paper Zhou et al., "Simultaneous System Identification and Model Predictive Control with No Dynamic Regret", TRO 2025. We will build upon the MPC controller developed in Lab 3 (MPC for trajectory tracking) and extend it to incorporate online learning capabilities to handle unknown wind and inaccurate model parameters. We will use the same simulator for the Crazyflie quadrotor, and also the same `controller_pkg` ROS2 package that we developed in Lab 3. The new files for Lab 8 are provided in the `lab8` folder of the `ae740_labs` repository.

## Instructions for using the files 
1. Pull the latest version of `ae740_labs` repository to get the Lab 8 files (is the `lab8` folder): `git pull origin main`.
2. Copy the `crazyflie_ssi_mpc_node.py` (equivalent to `crazyflie_mpc.py`) and `ssi_mpc.py` file (equivalent to `tracking_mpc.py`) from `lab8` folder to your existing `ros2_ws/src/controller_pkg/controller_pkg/` directory, that we used in Lab 3.  
3. Inside the `ros2_ws/src/controller_pkg` directory, create a new launch file for this lab (*not provided here*) and add the new node as an executable to the `setup.py` file. You can choose to use the `ros2 run` command instead of a launch file, since we are only launching one node here.

## Instructions for implementing the paper
1. Read the paper "Simultaneous System Identification and Model Predictive Control with No Dynamic Regret", Zhou et al., TRO 2025, particularly, Sections II and III, that describe the problem formulation and the SSI-MPC algorithm.
2. Section V-A  contains the details about the actual implementation and parameters used for the SSI-MPC algorithm.
3. The main implementation of *Algorithm 1*, in the paper is inside the `ssi_mpc.py` file, using the function `update_step()`. 
4. The `crazyflie_ssi_mpc_node.py` file contains the initialization of the parameters for the SSI-MPC algorithm, which are passed to the main solver object of class `SSIMpc` defined in `ssi_mpc.py`.
5. Read throught the comments and code structure in both files carefully before starting the implementation. For better understanding, it is also recommended to compare with the `tracking_mpc.py` and `crazyflie_mpc.py` files from Lab 3 to understand how new functionalities are added for SSI-MPC.


## Comments on Implementation details
1.  SSI-MPC requires modification to the MPC model, to include the learned prediction model for solving the optimal control problem. Hence, we incorporate this inside the `ssi_mpc.py` file, inside the `ssi_augmented_model()` function, instead of the `quadrotor_simplified_model.py` file used in Lab 3.
2. We use the non-linear function $\phi(\cdot)$ in the paper, as a *cosine* function. 
3. Based on comment Sec. V-A-4 in the paper, we do not use any function $B(w)$. Additionally, we also skip the *Algorithm 1: step 9* that involves the projection of $\alpha$.
4. The function `update_step()` in `ssi_mpc.py` essentially assumes the control input from the previous MPC iteration (*Algorithm 1: step 4*). Hence, it contains the *steps 5-8* from the *Algorithm 1*.
5. It is recommended to read carefull through the variable definitions in the `__init__()` functions and take a note of the dimensions of the variables used in the implementation. You can then use to implement the algorithm on the paper first, using the same variable names for better clarity.
6. For this assignment, we fix the MPC horizon to $N=10$ and the time horizon of $1s$. The MPC is solved at the control update rate of $50 Hz$.
7. At the end, you might require tuning for the learning rate and standard deviation of the Gaussian kernel to get good performance. 
8. You will need to use the code from Lab 3 to fill the parts of the code that are not related to SSI-MPC, in the given files. It follows the same structure as Lab 3, and you must copy the relevant parts from your Lab 3 implementation.
9. You can use low costs for position error to see the effect of wind disturbances more clearly in the nominal MPC. You can then use the same low costs for SSI-MPC as well, to compare the performance. This is to observe the effect of learning in SSI-MPC, since high costs can mask the effect of wind disturbances in the nominal MPC.

## Instructions for adding wind disturbances in the simulator
1. To add wind disturbances in the Gazebo simulator, you need to modify the world file `ae740_crazyflie_sim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/worlds/crazysim_default.sdf`. Add the following line inside the `<world>` tag near the `<atmosphere>` tag:
```xml
        <!-- Other code -->
        <gravity>0 0 -9.80665</gravity>
        <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
        <atmosphere type='adiabatic'/>
        <!-- Add wind disturbance here -->
        <wind>
            <linear_velocity>1 1 0</linear_velocity>
        </wind>
        <!-- Other code -->
```

You can modify the `<linear_velocity>` values to change the wind speed and direction as needed.

2. Enable the wind in the quadrotor model `ae740_crazyflie_sim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie/model.sdf` and `ae740_crazyflie_sim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie/model.sdf.jinja` **(BOTH)**, by adding the following line inside the `<link name='base_link'>` tag:
```xml
    <!-- Other code -->
    <model name="crazyflie">
        <enable_wind>true</enable_wind> <!-- Add this line -->
        <pose>0 0 0 0 0 0</pose>
        <static>false</static>
        <!-- Other code -->
```

## Deliverables 
1. **Mathematical Interpretation (10 pts):** Provide a brief explanation of how the SSI-MPC algorithm works in practical terms, based on your implementation. Include brief mathematical descriptions of the key steps involved that you implemented. 
2. **Wind Resiliency (50 pts):** Demonstrate the performance of your SSI-MPC controller in the presence of wind disturbances. Provide at least two different wind scenarios (e.g., different wind speeds and directions) and show how your controller adapts to these conditions. Include plots of the drone's states (X-Y-Z) for SSI-MPC and nominal MPC (from Lab 3), and plot errors to compare the effectiveness. Use the `lemniscate` trajectory for this demonstration.

*Note that high-values of wind speed (e.g., > 3 m/s) might practically cause crash immediately after the take-off. This is due to Gazebo Sim, rather than SSI-MPC.*

3. **Model Inaccuracy Handling (20 pts):** Test the SSI-MPC controller's ability to handle model inaccuracies. Introduce discrepancies in the quadrotor's mass and or deliberate corruption of model (say replacing $sin()$ with $cos()$ in the model) in the MPC model. Evaluate how well the SSI-MPC controller compensates for these inaccuracies during flight compared to the same discrepancies in the nominal MPC. Provide plots of the drone's states (X-Y-Z) and difference between errors of both SSI-MPC and nominal MPC (from Lab 3) using any **3D** trajectory of your choice. Also briefly mention the modifications you made to introduce model inaccuracies.

**Final Submissions:** 1. The completed `crazyflie_ssi_mpc_node.py` and `ssi_mpc.py` code files. 2. PDF report including answers to the deliverables, plots, explanations and copy of the code. 3. Video files demonstrating the performance for each deliverable.

## Reference
Zhou, Hongyu, and Vasileios Tzoumas. "Simultaneous system identification and model predictive control with no dynamic regret." IEEE Transactions on Robotics (2025).
 [Link](https://arxiv.org/pdf/2407.04143)
