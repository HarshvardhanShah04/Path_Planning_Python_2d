# Dynamic Window Approach (DWA) - Python

This project is a simple implementation of the **Dynamic Window Approach (DWA)** for robot path planning and obstacle avoidance using Python.

The robot starts from an initial position and tries to reach a given goal while avoiding circular obstacles.

## How it Works

DWA is a local path planning algorithm which works by predicting different possible trajectories of the robot and selecting the best one.

The algorithm follows these steps:

1. **Dynamic Window:** Calculate the possible linear and angular velocities based on the robot's acceleration and velocity limits.
2. **Trajectory Prediction:** Generate different trajectories using these velocities and the robot's kinematic model.
3. **Cost Calculation:** Evaluate each trajectory using a cost function.
4. **Select Best Trajectory:** Choose the trajectory having the minimum cost.
5. **Update:** Move the robot using the selected velocity and repeat the process until it reaches the goal.

## Cost Function and Heuristics

The algorithm uses four different heuristics to evaluate the predicted trajectories.

### 1. Goal Cost

This cost checks how much progress the robot makes towards the goal.

The initial and final distances from the goal are compared. Trajectories which bring the robot closer to the goal are preferred.

### 2. Heading Cost

This cost checks whether the robot is facing towards the goal at the end of the predicted trajectory.

The angular difference between the robot's final heading and the direction towards the goal is calculated.

A smaller heading error gives a lower cost.

### 3. Obstacle Cost

This cost is used for obstacle avoidance.

The minimum clearance between the robot and all obstacles is calculated along the predicted trajectory.

- Trajectories entering the safety region of an obstacle are rejected by assigning infinite cost.
- Trajectories passing close to obstacles are given a higher cost using a quadratic penalty.
- Trajectories maintaining sufficient clearance have zero obstacle cost.

This helps the robot maintain a safe distance from obstacles instead of simply avoiding collisions.

### 4. Velocity Cost

This cost encourages the robot to maintain a preferred linear velocity.

The difference between the sampled velocity and the preferred velocity is calculated.

This helps in avoiding unnecessarily high or low velocities while navigating.

### Total Cost

All four costs are combined using a weighted cost function:

$$
J = \frac{w_g J_g + w_h J_h + w_o J_o + w_v J_v}{w_g+w_h+w_o+w_v}
$$

Where:

- `wg` = Goal cost weight
- `wh` = Heading cost weight
- `wo` = Obstacle cost weight
- `wv` = Velocity cost weight

The weights decide the importance of each heuristic.

For example, increasing the obstacle weight makes the robot prefer trajectories with more clearance, while increasing the goal weight gives more importance to reaching the goal.

The trajectory having the **minimum total cost** is selected.

## Simulation

- Robot is modelled using simple unicycle kinematics.
- Obstacles are represented as circles.
- Euler integration is used for predicting the trajectories and final simulation.
- Matplotlib is used to animate the robot's movement.
- Robot parameters, obstacles and cost weights can be changed in the code.
- The algorithm continuously recalculates the best velocity commands as the robot moves.

## Requirements

- Python 3
- NumPy
- Matplotlib

Install the required libraries:

```bash
pip install numpy matplotlib
```

## Demonstration Videos

### Vd-1

[![DWA Demonstration 1](https://img.youtube.com/vi/KO4RWN-qZzg/hqdefault.jpg)](https://youtu.be/KO4RWN-qZzg)

### Vd-2

[![DWA Demonstration 2](https://img.youtube.com/vi/lMCKuKTVJE8/hqdefault.jpg)](https://youtu.be/lMCKuKTVJE8)

### Vd-3

[![DWA Demonstration 3](https://img.youtube.com/vi/3VjldT42abc/hqdefault.jpg)](https://youtu.be/3VjldT42abc)

## Limitations

- DWA is a local path planning algorithm and does not have information about the complete environment.
- The robot may get stuck in local minima, especially in complex obstacle arrangements.
- The performance depends on the selected cost weights, prediction horizon and sampling resolution.
- This implementation assumes static circular obstacles.

This project is mainly developed for understanding the working of DWA and experimenting with different cost functions and heuristics.
