---
layout: default
title: "Exercises"
parent: "Lab 9"
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

**Deadline:** To submit your solution, please upload the corresponding files under `Assignment > Lab 9` by **Tue, Nov 25 11:59 EST**.


## Overview
In this lab, we will implement a Self-Adaptive Learning framework for target pursuit scenarios, based on Lab 4. The goal is to develop an MPC controller that can **adaptively select between multiple expert predictors** to track a moving target with unknown motion dynamics. This builds upon the concepts from Lab 8 (SSI-MPC) by incorporating multiple learned models and an adaptive selection strategy. The implementation is based on the paper "Self-Adaptive Learning and Model Predictive Control for Tracking Unknown Dynamics with No Regret", presented in the lecture.

In this scenario, one Crazyflie (`cf_2`) acts as the **pursuer** that must track another Crazyflie (`cf_1`) acting as the **target**. The pursuer uses self-adaptive MPC with multiple expert predictors to anticipate and follow the target's trajectory.


## Instructions for using the files 
1. Pull the latest version of `ae740_labs` repository to get the Lab 9 files (in the `lab9` folder): `git pull origin main`.
2. Copy the `crazyflie_self_adap_mpc_node.py` and `self_adaptive_mpc.py` files from the `lab9` folder to your existing `ros2_ws/src/controller_pkg/controller_pkg/` directory.
3. Inside the `ros2_ws/src/controller_pkg` directory, add the new node as an executable to the `setup.py` file. You can choose to use the `ros2 run` command or create a launch file to run the node.
4. **Important:** You will need to launch **two** Crazyflie drones in the simulator - one as the target (`cf_1`) and one as the pursuer (`cf_2`). The target drone should be running an MPC controller that you developed in previous labs.


## Instructions for implementing the paper
1. Read the paper *"Self-Adaptive Learning and Model Predictive Control for Tracking Unknown Dynamics with No Regret"*, presented in the lecture, particularly Algorithms 1, 2 and 3 that describe both the Isolated Online Learning *(ISO)* and Adaptive Selection *(AS)* modules.
2. Section VII contains implementation details and experimental parameters used for the self-adaptive MPC algorithm.
3. The main implementation consists of three key components:
   - **Expert Initialization:** Initialize multiple MPC experts with different learning parameters (kernel std, learning rate, memory horizon)
   - **Online Learning (ISO):** Each expert learns to predict the target's motion using recursive least squares (`update_step()` function in `self_adaptive_mpc.py`)
   - **Adaptive Selection (AS):** Dynamically select the best-performing expert based on prediction accuracy (`adaptive_expert_selection()` function in `crazyflie_self_adap_mpc_node.py`)
4. The `crazyflie_self_adap_mpc_node.py` file contains the initialization of multiple experts and the adaptive selection strategy.
5. The `self_adaptive_mpc.py` file contains the individual expert implementation, including the learning update and prediction error calculation.
6. Read through the comments and code structure in both files carefully. 


## Comments on Implementation Details
1. **Multiple Experts:** Unlike Lab 8 which used a single learned model, Lab 9 initializes multiple expert predictors with different hyperparameters.
2. **Memory Horizon:** Each expert uses a **memory horizon** (e.g., 3-5 past positions) of the target to predict future positions. This is different from Lab 8 which used current state only. These past positions are spaced same as the MPC time step (0.1s), not the controller update rate (50 Hz -> 0.02s).
3. **Random Features:** Similar to Lab 8, we use Random Fourier Features (RFF) with cosine activation functions. Each expert has its own set of random features (w, b). The update step arithmetic is similar to Lab 8, but now done with different set of features for each expert.
4. **Adaptive Selection Module:** The AS module maintains a probability distribution over experts and updates it based on prediction errors. Variable `selection_interval` determines how often to update expert selection (since this doesn't happen every iteration). The choice of reward (or loss) function are open to experimentation, or can be kept same as the paper.
5. **Prediction Error Metric:** Expert performance is evaluated based on the RMS error between predicted and actual target **velocities** over the MPC horizon. While you can also consider position errors, the prediction model ISO is designed to predict velocities, hence using velocity errors is more consistent.
6. **MPC Parameters:** Fix MPC horizon to N=10, time horizon of 1s, and control update rate of 50 Hz (consistent with Lab 8).
7. **Target Memory Vector:** Each expert constructs a feature vector from the past `mh` target positions (spaced at intervals based on the update rate). You will need to be careful in extracting the correct past positions from the dense data received at 50 Hz. Importantly, the positions must be converted to a 1D vector consistently across all functions inside `self_adaptive_mpc.py`.
8. **Suggested Workflow:** Since there are multiple components to implement, we suggest that you first focus on getting a single (or multiple with same parameters) expert/predictor working correctly. The prediction from the algorithm, also visible on RViz, should match the target trajectory reasonably well. After finding a reasonable set of hyperparameters for one expert, you can then expand to multiple experts to see if the AS module converges to the *known* best expert over time.
9. You will need to use code from previous labs (Lab 3 and Lab 8) to fill in the ROS2 subscriber/publisher setup and basic MPC components. Make sure you disable the wind disturbance in the simulator for this lab.


## Deliverables 
1. **Algorithm Explanation (15 pts):** Provide a brief overview of the self-adaptive MPC algorithm that *you* implemented, focusing on:
   - Learning strategy/step for single predictor.
   - Prediction error evaluation for a single expert.
   - Mathematical description of the RLS update and adaptive selection strategy.

2. **Target Tracking Performance (40 pts):** Demonstrate your self-adaptive MPC controller tracking a moving target drone. Provide results for at least **two different target trajectories** (e.g., circular, lemniscate etc.):
   - Plot tracking errors for the overall algorithm over time, in comparison to a nominal MPC without learning (just tracking the last known position).
   - Include video of the pursuer correctly tracking and predicting the learned target trajectory

3. **Expert Comparison (25 pts):** Analyze the performance of different expert configurations:
   - Compare at least 3 different expert configurations (varying kernel std, learning rate, or memory horizon)
   - Plot prediction errors for each expert over time, when used separately.
   - Show how the AS module adapts to changing target behavior, in terms of expert selection probabilities.


**Final Submissions:** 
1. The completed `crazyflie_self_adap_mpc_node.py` and `self_adaptive_mpc.py` code files.
2. PDF report including answers to all deliverables, plots, analysis, and copy of the code.
3. Video files demonstrating the target tracking scenarios based on deliverables.


## Bonus (Optional)
Implement the **multiple learners** predictor type (`p_type='multiple_learners'`) instead of the single learner. In this approach, each target position in the MPC horizon has its own separate learner (as shown in the paper). In this case, the updated step will have different target memory vector size based on how far we move ahead in horizon. This typically shows improved performance compared to the single learner approach.


