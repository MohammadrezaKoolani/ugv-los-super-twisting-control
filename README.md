# UGV LOS Super-Twisting Control

Implementation of line-of-sight path-following guidance and super-twisting surge/heading controllers for an unmanned ground vehicle.

Minimal Python implementation of:

- LOS path-following guidance
- super-twisting surge speed control
- super-twisting heading/yaw-rate control

## Structure

- `guidance/los.py`: LOS path-following guidance
- `controllers/surge.py`: surge speed controller
- `controllers/heading.py`: heading controller
- `models/state.py`: states, parameters, and configs
- `types.py`: outputs and path segment data structures

## Notes

This is a first clean implementation focused on controller structure.

It is not yet a full physical simulator.