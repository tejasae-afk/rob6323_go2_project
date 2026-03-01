# ROB6323 – Stable and Smooth Locomotion for Unitree Go2

This repository contains our solution to the ROB6323 quadruped locomotion project using Isaac Lab and the Unitree Go2 robot. The objective is to train a reinforcement learning (RL) policy that produces stable, smooth, and natural walking or trotting gaits while accurately tracking commanded linear and angular velocities.

The implementation follows the official tutorial (Parts 1–4) and extends it with additional reward shaping, regularization, and a bonus actuator friction model to satisfy the full grading criteria.

To get started, follow the tutorial as provided in Project Tutorial: https://github.com/machines-in-motion/rob6323_go2_project/blob/master/tutorial/tutorial.md

This project was submitted by Ankush Pratap Singh (ax2047) and Tejas Attarde (ta2867)

---

## Project Goals

The trained policy is designed to:

- Track commanded planar velocities (vx, vy, yaw rate) with low steady-state error  
- Produce a clear walking or trotting gait (no pacing or hopping)  
- Maintain base stability (low roll/pitch oscillations, reasonable height)  
- Generate smooth actions and torques  
- Avoid catastrophic failures when commands change slowly  

---

## Baseline: Tutorial Parts 1–4

The following components are implemented exactly as described in the tutorial.

### Part 1 – Action Smoothness
- Maintains a 3-step action history buffer
- Penalizes first and second finite differences of actions:
  - ||a_t − a_{t−1}||²
  - ||a_t − 2a_{t−1} + a_{t−2}||²

### Part 2 – Explicit PD Torque Control
- Joint-space PD control:
  τ = Kp (q_des − q) − Kd q_dot
- Constants:
  - Kp = 20.0
  - Kd = 0.5
  - Action scale = 0.25
- Implicit actuator stiffness and damping are disabled

### Part 3 – Early Termination
Episodes terminate when:
- Base height < 0.20 m
- Robot flips upside down
- Excessive base contact is detected
- Episode timeout (20 seconds)

### Part 4 – Gait Clock and Raibert Heuristic
- Adds 4 sinusoidal gait clock inputs to observations
- Uses a Raibert-style heuristic to encourage periodic and symmetric foot placement

---

## Reward Refinement (Parts 5 and 6)

To address baseline limitations (bouncing, oscillations, torque spikes), the reward function was extended.

### Base Stability and Attitude (Part 5)

| Term | Definition | Weight |
|----|----|----|
| Orientation penalty | g_bx² + g_by² | −5.0 |
| Vertical velocity | z_dot² | −0.02 |
| Roll/pitch angular velocity | ω_x² + ω_y² | −0.001 |
| Joint velocity | Σ q_dot² | −1e−4 |

These penalties reduce roll/pitch oscillations and vertical bouncing while keeping the base approximately parallel to the ground.

---

### Action and Torque Smoothness

- Torque L2 penalty:
  Σ τ²
- Weight: −1e−4 (as recommended in the grading rubric)
- Torque clipping:
  τ_max = 100.0

This significantly improves visual smoothness without degrading tracking performance.

---

### Gait Shaping and Foot Interaction (Part 6)

| Term | Description | Weight |
|----|----|----|
| Foot clearance | Penalizes low foot height during swing | −30.0 |
| Contact tracking | Matches contact forces to gait schedule | +4.0 |

Constants:
- Target foot clearance: 0.08 m
- Contact force normalization: 50 N

Correct indexing is used to distinguish robot body indices from contact sensor indices.

---

## Command Following

Command tracking uses exponential rewards:

- Linear velocity tracking:
  exp(−||v_xy_cmd − v_xy||² / 0.25)
- Yaw rate tracking:
  exp(−(yaw_rate_cmd − yaw_rate)² / 0.25)

Both are multiplied by the control timestep (dt_c = 0.02 s) so that TensorBoard values remain comparable to tutorial references:

- track_lin_vel_xy_exp ≈ 48
- track_ang_vel_z_exp ≈ 24

Visual confirmation is provided using velocity arrows (green = command, blue = actual).

---

## Bonus: Actuator Friction Modeling

As a bonus extension, an actuator friction model was implemented:

τ_applied = τ_PD − (F_s tanh(q_dot / 0.1) + μ_v q_dot)

Per-episode randomization:
- μ_v ~ Uniform(0, 0.3)
- F_s ~ Uniform(0, 2.5)

This encourages robustness to actuator uncertainty and improves realism.

---

## Environment and Training Setup

- Simulation timestep: 0.005 s
- Control decimation: 4
- Control period: 0.02 s
- Episode length: 20 s
- Action dimension: 12

### Command Sampling
- vx ~ Uniform(−1.0, 1.0)
- vy ~ Uniform(−0.6, 0.6)
- yaw_rate ~ Uniform(−1.0, 1.0)

---

## Running Training

For setting up, running HPC and installing IsaacLab, follow the Project Page: https://github.com/machines-in-motion/rob6323_go2_project/tree/master

From the repository root run: ./train.sh

To check the status of the job: ssh burst "squeue -u $USER"

## Monitor Training (TensorBoard)

Download logs to your computer: rsync -avzP -e 'ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null' <netid>@dtn.hpc.nyu.edu:/home/<netid>/rob6323_go2_project/logs ./

On the local machine (Assuming Tensorboard is installed) run: tensorboard --logdir logs

## Expected Outcome
After training:
- Stable base height (no dragging or frequent collapses)
- Low roll/pitch oscillations
- Smooth joint torques (no aggressive spikes)
- Accurate command following under slowly changing commands
