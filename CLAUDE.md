# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a MATLAB-based inverted pendulum control simulation implementing fuzzy logic control (Fuzzy Control) for stabilizing an inverted pendulum system. The project demonstrates a cart-pendulum system where a pendulum is balanced upright on a moving cart through force control.

## How to Run

### Main Simulation
Run the fuzzy control simulation:
```matlab
inverted_pendulum_Fuzzy
```

### Alternative Control Method
Run the PID control comparison:
```matlab
cd sub_functions
inverted_pendulum_PID
```

Before running, ensure MATLAB is in the project root directory or the `sub_functions` directory is in the MATLAB path.

## Architecture

### Control Flow (Fuzzy Control)
The fuzzy control system follows a three-stage pipeline executed at each control timestep (Ts = 0.01s):

1. **Fuzzification** (`fuzzification.m`): Converts normalized error signals (cart position error `ex`, pendulum angle error `eq`, and their derivatives) into fuzzy membership values
   - Uses 5 fuzzy sets: NB (Negative Big), NS (Negative Small), ZO (Zero), PS (Positive Small), PB (Positive Big)
   - Returns a 5x2 matrix of membership values for error and error derivative

2. **Fuzzy Inference** (`fuzzy_inference.m`): Applies fuzzy rules to determine control action
   - Uses a 5x5 fuzzy rule matrix to map input fuzzy sets to output fuzzy sets
   - Returns weighted fuzzy set indices based on rule activation

3. **Defuzzification** (`defuzzification.m`): Converts fuzzy output to crisp control force
   - Uses center of gravity method over 100-point discretization
   - Returns normalized control value `u_tilde` in range [-1, 1]

### Dual-Controller Architecture
The system uses **two independent fuzzy controllers** running in parallel:
- **Cart position controller**: Generates `u_x` to control cart position
- **Pendulum angle controller**: Generates `u_q` to stabilize pendulum angle
- Final control force: `F = -u_x + u_q` (inverted_pendulum_Fuzzy.m:57)

### Dynamics Model
The physical system is modeled using Lagrangian mechanics in `diff_pendulum.m`, which computes state derivatives for the 4-state system:
- State vector: `X = [cart_position; cart_velocity; pendulum_angle; angular_velocity]`
- Solved numerically using MATLAB's `ode45` integrator

### Normalization Strategy
The control system uses normalization factors to map physical quantities to the fuzzy domain [-1, 1]:
- Cart position: 12 m
- Cart velocity: 1.5 m/s
- Pendulum angle: 360° (2π rad)
- Pendulum angular velocity: 180° (π rad)
- Control force: 1000 N (with separate gains for u_x and u_q)

## System Parameters
Key physical parameters defined globally:
- M = 2 kg (cart mass)
- m = 0.8 kg (pendulum mass)
- l = 0.25 m (pendulum length)
- bx = 0.005 kg/s (cart damping)
- bq = 0.0005 kg·m²/s (pendulum damping)
- J = 0.0326 kg·m (moment of inertia)

## Visualization
The main script produces:
- Figure 1: Real-time animation of the cart-pendulum system (optional video recording with `video_flag = 1`)
- Figure 2: Three subplots showing time histories of cart states, pendulum states, and control forces

## Code Modifications
When modifying control parameters:
- Fuzzy normalization factors: Lines 28-33 in inverted_pendulum_Fuzzy.m
- Initial conditions: Line 21 in inverted_pendulum_Fuzzy.m
- Fuzzy rule matrix: Lines 8-12 in fuzzy_inference.m
- Membership function shapes: Lines 17-33 in fuzzification.m and switch statement in defuzzification.m

When modifying physical parameters, update the global variable definitions in the main script (inverted_pendulum_Fuzzy.m:6-15) - these are shared with diff_pendulum.m.
