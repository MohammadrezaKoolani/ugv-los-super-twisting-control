# UGV LOS Super-Twisting Control

Python implementation of line-of-sight (LOS) path-following guidance and super-twisting surge and heading controllers for an unmanned ground vehicle (UGV).

This project is based on the controller structure from the paper:

**von Ellenrieder, Licht, Belotti, Henninger (2022)**  
*Shared human–robot path following control of an unmanned ground vehicle*

In this repository, the focus is only on:

- **surge speed control**
- **LOS path-following / heading control**

The shared-control / human-input part of the paper is intentionally not implemented.

---

## What is implemented

This project currently includes:

- waypoint-based piecewise-linear path following
- waypoint switching using an acceptance radius
- LOS guidance law
- super-twisting surge controller
- super-twisting heading controller
- controller manager that connects all modules
- parameter sets for vehicle type:
  - 8-wheel vehicle
- basic tests
- example scripts for:
  - one-step controller check
  - simple closed-loop simulation
  - plotted simulation results

---

## Implemented control structure

The implementation follows these parts of the paper:

### Vehicle/path-following side

- kinematic quantities and path geometry
- cross-track error
- LOS desired heading
- desired yaw-rate generation

### Control side

- surge tracking error
- super-twisting surge controller
- heading/yaw-rate tracking error
- super-twisting heading controller
- combined machine control output

### Included paper equations

The implementation is organized around the paper equations up to:

- **surge control:** Eq. (11), Eq. (16)
- **LOS guidance:** Eq. (21), Eq. (24), Eq. (27), Eq. (29), Eq. (33), Eq. (41)
- **heading control:** Eq. (44)
- **combined machine control:** Eq. (48)
- **waypoint switching:** Eq. (10)

For the current practical implementation, the LOS guidance uses the same simplification used in the paper’s experiments:

- $\beta = 0$
- $\dot{\beta} = 0$

So the desired heading is implemented as:

$$
\psi_d = \alpha_k + \tan^{-1}\left(-\frac{e}{\Delta}\right)
$$

and the desired yaw rate as:

$$
r_d = -\frac{\Delta}{e^2 + \Delta^2}\dot{e}
$$

---

## Repository structure

```text
ugv-los-super-twisting-control/
├── examples/
│   ├── basic_path_following_demo.py
│   ├── simple_kinematic_sim.py
│   ├── simple_dynamic_sim.py
│   └── plot_simple_dynamic_sim.py
├── src/
│   └── ugv_control/
│       ├── controllers/
│       │   ├── heading.py
│       │   ├── manager.py
│       │   └── surge.py
│       ├── guidance/
│       │   ├── los.py
│       │   ├── path.py
│       │   └── waypoint_manager.py
│       ├── interfaces/
│       │   ├── command_mapper.py
│       │   ├── eight_wheel_mapper.py
│       │   └── four_wheel_mapper.py
│       ├── models/
│       │   ├── states.py
│       │   ├── vehicle_4ws.py
│       │   ├── vehicle_8ws.py
│       │   └── vehicle_base.py
│       ├── __init__.py
│       ├── types.py
│       └── utils.py
├── tests/
│   └── test_basic_controller.py
└── README.md
```

---

## Main modules

### `guidance/los.py`

Computes LOS guidance quantities for the active path segment:

- path angle
- cross-track error
- cross-track error derivative
- desired course
- desired heading
- desired yaw rate

### `guidance/waypoint_manager.py`

Updates the active segment index using the waypoint acceptance rule.

### `guidance/path.py`

Provides helper functions for extracting the active path segment.

### `controllers/surge.py`

Implements the super-twisting surge controller.

### `controllers/heading.py`

Implements the super-twisting heading controller.

### `controllers/manager.py`

Connects:

- waypoint update
- active segment extraction
- LOS guidance
- surge controller
- heading controller

and returns the combined machine control output.

### `models/states.py`

Defines runtime state containers:

- vehicle state
- waypoint progress
- controller internal states

### `models/vehicle_base.py`

Defines the common parameter structure for vehicles and controllers.

### `models/vehicle_4ws.py`

Default parameter set for a 4-wheel vehicle.

### `models/vehicle_8ws.py`

Default parameter set for an 8-wheel vehicle.

### `types.py`

Defines path, waypoint, segment, LOS output, and controller output data structures.

---

## Current status

This repository is currently focused on controller structure and path-following behavior.

It is not yet a full physical implementation of the paper’s complete 3-DOF vehicle dynamics.

### What is currently included

- controller implementation
- waypoint/path logic
- simplified closed-loop simulation
- plotting tools
- basic tests

### What is not yet included

- shared human–robot blending
- joystick/human intent model
- full actuator model
- full 3-DOF dynamic simulation with sway dynamics
- ROS integration
- real vehicle communication layer

---

## Simulation notes

### `plot_simple_dynamic_sim.py`

Runs the simplified dynamic simulation and plots:

- trajectory vs path
- cross-track error
- speed response
- controller outputs
- heading tracking

For the current simplified simulation setup, a larger lookahead distance was found to give smoother path convergence than the original small value.

---

## Installation / requirements

This project currently assumes a simple local Python environment.

### Required packages

- Python 3.10+
- `matplotlib`
- `pytest`

Install dependencies with:

```bash
python3 -m pip install matplotlib pytest
```

---

## How to run

Run commands from the repository root.

Because the package lives under `src/`, set `PYTHONPATH=src` when running scripts.

### 1. Basic one-step controller check

```bash
PYTHONPATH=src python3 examples/basic_path_following_demo.py
```

This runs one controller step and prints:

- active segment
- cross-track error
- desired heading
- desired yaw rate
- surge command
- heading command

### 2. Simple kinematic simulation

```bash
PYTHONPATH=src python3 examples/simple_kinematic_sim.py
```

This is a lightweight structural test.

### 3. Simplified dynamic simulation

```bash
PYTHONPATH=src python3 examples/simple_dynamic_sim.py
```

This runs a more realistic command-level closed-loop simulation and prints the vehicle state over time.

### 4. Plot the simulation results

```bash
PYTHONPATH=src python3 examples/plot_simple_dynamic_sim.py
```

This opens plots for:

- vehicle trajectory
- cross-track error
- speed response
- controller outputs
- heading tracking

### 5. Run tests

```bash
PYTHONPATH=src python3 -m pytest tests
```

or:

```bash
PYTHONPATH=src python3 -m pytest tests/test_basic_controller.py
```

Current basic tests check:

- cross-track error computation
- LOS desired heading direction
- positive surge command when current speed is below desired speed
- waypoint switching inside the acceptance radius

---

## Example behavior

With the current simplified dynamic simulation and tuned lookahead distance, the controller is able to:

- steer the vehicle toward the path
- reduce cross-track error smoothly
- regulate surge speed close to the desired value
- switch to the next path segment when entering the waypoint acceptance radius

---

## Design philosophy

This repository is organized so the same path-following controller can be reused on more than one vehicle.

The control pipeline is vehicle-agnostic:

- path and waypoint logic
- LOS guidance
- surge controller
- heading controller
- combined control output

Vehicle-specific differences are isolated in:

- parameter files
- interface / command mapping layer

This makes it easier to support multiple vehicle platforms such as:

- 4-wheel UGV
- 8-wheel UGV

---

## Next development directions

Planned or possible future improvements include:

- fuller dynamic simulation
- sway dynamics and side-slip effects
- command mapper refinement for real vehicles
- ROS/ROS 2 interface
- trajectory logging to file
- controller tuning tools
- support for loading parameters from YAML or JSON
- human/shared-control extension from the paper

---

