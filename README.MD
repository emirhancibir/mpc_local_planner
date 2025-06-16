# MPC Local Planner for Differential Drive Robot

This package implements a **Model Predictive Control (MPC)** based local planner for a differential drive robot using the **CasADi** optimization framework. It is designed to be used as a plugin within the ROS `move_base` navigation stack.

---

## TODO

### Core Functionality
- [ ] change goal_reached function
- [ ] Improve heading error calculation to reduce oscillations near the goal
- [ ] Incorporate obstacle cost into the MPC cost function using costmap values

## Features

* Implements nonlinear MPC using CasADi and IPOPT
* Optimizes over forward velocity `v` and angular velocity `w`
* Penalizes:

  * Position error to reference trajectory
  * Control effort (v and w)
  * Heading (orientation) error
* Uses ROS `costmap_2d::Costmap2DROS` and TF for robot state
* Compatible with `nav_core::BaseLocalPlanner` interface

---

## Dependencies

* ROS (tested on Noetic)
* CasADi (installed from source or via pip)
* `pluginlib`
* `geometry_msgs`, `tf2`, `costmap_2d`

---

## Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/emirhancibir/mpc_local_planner
cd ..
catkin_make
```

---

## Usage

1. Add the planner to your `move_base` config:

```yaml
base_local_planner: "mpc_local_planner/MPCPlanner"
```

2. Ensure CasADi is installed and linked properly in `CMakeLists.txt` and `package.xml`.

3. Launch with `move_base` and send a goal in `map` frame.

---

## Behavior

### Cost Function

The MPC minimizes the following cost at each time step `k`:

```
J = ∑ (||x_k - ref_k||^2 + lambda1 * ||u_k||^2 + lambda2 * heading_error_k^2)
```

Where:

* `x_k` is the robot state \[x, y, theta]
* `u_k` is the control input \[v, w]
* `ref_k` is the reference waypoint
* `heading_error_k` is the orientation error (wrapped in \[-pi, pi])

### Dynamics

```cpp
x_next = x + v * dt * cos(theta)
y_next = y + v * dt * sin(theta)
theta_next = theta + w * dt
```

---

## Notes

* The robot is expected to move **forward only** (`v` in \[0, vmax]).
* Angular velocity is symmetric (`w` in \[-wmax, +wmax]).
* Heading error is wrapped properly to avoid oscillations using CasADi-safe modulo logic.

---

