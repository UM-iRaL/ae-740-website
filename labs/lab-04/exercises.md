---
layout: default
# published: false # not released yet (2026)
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

### Individual

Create a folder called `lab4` that includes your answers (for math-related questions LaTeX is preferred but handwritten is accepted too). Zip the folder and upload on Canvas.

Each student needs to submit their own `lab4` folder to Canvas.

### Team

Each group should create a new folder called `TEAM_<N>` replacing `<N>` with your team number. For example team 2 will name their folder `TEAM_2`. Please put the source code of the entire `controller_pkg` package in the folder `TEAM_2`. Zip the folder and submit to Canvas.

Each team will only need to submit one `TEAM_<N>.zip` to Canvas.

**Additional Instructions:** For organizing the submission better, please include a single report (PDF format) inside your folder that includes your answers, explanations, video file names and **copy of the Python code** that you edit/write. Please make sure to include your team member names and IDs in the report, and follow this format for future lab submissions as well. 

**Deadline:** To submit your solution, please upload the corresponding files under `Assignment > Lab 4` by **Wed, Oct 14 11:59 EST**.


## Individual

    
### Deliverable 1 - Modelling and control of UAVs (20 pts)


#### _A_. Structure of quadrotors
{: .no_toc}


<p align="center">
    <img src="../../../assets/img/lab4/drone_spinning.png" alt="Two drones example" style="width:50%;">
</p>


The figure above depicts two quadrotors _(a)_ and _(b)_. Quadrotor _(a)_ is a fully functional UAV, while for Quadrotor _(b)_ someone changed propellers 3 and 4 and reversed their respective rotation directions.


Show mathematically that quadrotor _(b)_ is not able to track a trajectory defined in position $[x,y,z]$ and yaw orientation $\Psi$.


- {: .hint} Hint: write down the $F$ matrix (see lecture notes eq. (6.9)) for the two cases _(a)_ and _(b)_ and compare the rank of the two matrices


#### _B_. Control of quadrotors 
{: .no_toc}


Assume that empirical data suggest you can approximate the drag force (in the body frame) of a quadrotor body as:


\\[
  F^b = \begin{bmatrix} 0.1 & 0 & 0 \\\ 0 & 0.1 & 0 \\\ 0 & 0 & 0.2\end{bmatrix} (v^b)^2
\\]


With $(v^b)^2= [-v^b_x \mid v^b_x \mid ,\, -v^b_y \mid v^b_y \mid ,\, -v^b_z \mid v^b_z \mid]^T $, and $v_x$, $v_y$, $v_z$ being the quadrotor velocities along the axes of the body frame.


With the controller discussed in class (see referenced paper [^1]), describe how you could use the information above to improve the tracking performance. 


- {: .hint} Hint: the drag force introduced above is an additional term in the system’s dynamics, which the controller could compensate for explicitly...


## Team


### Trajectory tracking for UAVs
In this section, we are going to implement the Model Predictive Controller on the Crazyflie software-in-the-loop (SITL) simulator based on Gazebo. This also enables use to deploy our controller on the hardware experiments. 

#### Getting the code
We use the same simulator and `controller_pkg` package from Lab 3. Follow these steps to get the new files:

1. Pull the latest version of your lab repository. The `lab4` folder contains the MPC files that you need to edit (`crazyflie_mpc.py`, `tracking_mpc.py`, `quadrotor_simplified_model.py`) and an updated `setup.py`. Pull the latest version using:
    ```bash
    cd ~/ae740_labs
    git pull
    ```

2. Copy the new files into the controller package from Lab 3, and **replace** its `setup.py` with the one from `lab4` (it registers the new MPC controller node):
    ```bash
    cp ~/ae740_labs/lab4/crazyflie_mpc.py ~/ae740_labs/lab4/tracking_mpc.py ~/ae740_labs/lab4/quadrotor_simplified_model.py ~/ae740_crazyflie_sim/ros2_ws/src/controller_pkg/controller_pkg/
    cp ~/ae740_labs/lab4/setup.py ~/ae740_crazyflie_sim/ros2_ws/src/controller_pkg/
    ```


#### Installation
The MPC controller uses the Acados optimizer, which was not needed in Lab 3. Install Acados and its Python interface following the [Acados section](https://github.com/UM-iRaL/ae740_crazyflie_sim?tab=readme-ov-file#6-acados-mpc-controllers-only) of the simulator README, then rebuild the ROS2 workspace:
```bash
cd ~/ae740_crazyflie_sim/ros2_ws
colcon build --symlink-install
```



### Deliverable 2 - Model Predictive Control (MPC) (60 pts)

Your main task is to complete the MPC controller implementation in the provided template in `ae740_crazyflie_sim/ros2_ws/src/controller_pkg/controller_pkg`. There are three main files inside this directory.
1. `crazyflie_mpc.py`: This is the main ROS2 node that run the control loop and handle the state/control subscriber and publisher.
2. `tracking_mpc.py`: This file contains the MPC optimization problem description, that uses the `acados` to formulate the problem.
3. `quadrotor_simplified_model.py`: This file contains a simpified version of the quadrotor dynamics, which we use in the MPC. 

To ensure a smooth workflow and successful implementation, you can follow the recommended sequence below. Each file contains `[TODO]` sections with specific instructions for completion:
1. **Review the code structure** in `crazyflie_mpc.py` to familiarize yourself with the main objects and methods.
2. **Complete Parts 1–5** in `crazyflie_mpc.py`, addressing each `[TODO]` as indicated.
3. **Implement the translational dynamics** in the `[TODO]` section of `quadrotor_simplified_model.py`.
4. **Define constraints and initialize cost functions** in `tracking_mpc.py`.
5. **Finish Part 6** in `crazyflie_mpc.py` to complete the MPC solver loop.
6. Once your code is functional, **run the MPC node** and use a separate terminal to command take-off and trajectory. Adjust the cost function parameters as needed for optimal performance.


#### MPC Problem Formulation
The quadrotor dynamics are modeled as a discrete-time nonlinear system:

$$\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k)$$

where the state vector $\mathbf{x} \in \mathbb{R}^{9}$ and control input $\mathbf{u} \in \mathbb{R}^4$ are defined as:
$$\mathbf{x} = [x, y, z, v_x, v_y, v_z, \phi, \theta, \psi]^T\quad$$
$$\mathbf{u} = [\phi_{cmd}, \theta_{cmd}, \dot{\psi}_{cmd}, T]^T$$

The finite-horizon optimal control problem is formulated as:

$$\min_{\mathbf{u}_0,...,\mathbf{u}_{N-1}} \sum_{k=0}^{N-1} \left( \|\mathbf{x}_k - \mathbf{x}_{ref,k}\|_\mathbf{Q}^2 + \|\mathbf{u}_k - \mathbf{u}_{ref,k}\|_\mathbf{R}^2 \right) + \|\mathbf{x}_N - \mathbf{x}_{ref,N}\|_\mathbf{P}^2$$

subject to:

$$\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k), \quad k = 0,...,N-1$$

$$\mathbf{x}_0 = \mathbf{x}_{current}$$

$$\mathbf{x}_{min} \leq \mathbf{x}_k \leq \mathbf{x}_{max}, \quad k = 0,...,N$$ 

$$\mathbf{u}_{min} \leq \mathbf{u}_k \leq \mathbf{u}_{max}, \quad k = 0,...,N-1$$

where:  
- $\|\mathbf{v}\|_\mathbf{W}^2 = \mathbf{v}^T\mathbf{W}\mathbf{v}$ is the weighted squared norm
- $\mathbf{Q} \in \mathbb{R}^{9 \times 9}$, $\mathbf{R} \in \mathbb{R}^{4 \times 4}$, and $\mathbf{P} \in \mathbb{R}^{9 \times 9}$ are positive definite weight matrices
- $N$ is the prediction horizon with time step $\Delta t$ s



#### Expected Deliverables

1. Completed `mpc_controller.py`, `tracking_mpc.py` and `quadrotor_simplified_model.py` file with your controller implementation. Include the `controller_pkg` package that contains all these files. 
2. A video demonstation (screen capture or video) of the quadrotor taking off and moving in a circular trajectory. 


Include all your code in the `TEAM_<N>` folder as specified in the submission instructions.

Please reach out during office hours or post on Piazza for any bugs, errors or code-related questions!


# References
[^1]: Lee, Taeyoung, Melvin Leoky, N. Harris McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." Decision and Control (CDC), 49th IEEE Conference on. IEEE, 2010 [Link](http://math.ucsd.edu/~mleok/pdf/LeLeMc2010_quadrotor.pdf)
