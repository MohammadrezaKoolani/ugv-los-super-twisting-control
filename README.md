# UGV LOS Super-Twisting Control

Python implementation of line-of-sight (LOS) path-following guidance and super-twisting surge/heading controllers for an unmanned ground vehicle (UGV).

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
- parameter sets for two vehicle types:
  - 4-wheel vehicle
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

- \( \beta = 0 \)
- \( \dot{\beta} = 0 \)

So the desired heading is implemented as:

\[
\psi_d = \alpha_k + \tan^{-1}\left(-\frac{e}{\Delta}\right)
\]

and the desired yaw rate as:

\[
r_d = -\frac{\Delta}{e^2 + \Delta^2}\dot{e}
\]

---

