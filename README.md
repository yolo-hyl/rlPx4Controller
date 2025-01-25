# rlPx4Controller

rlPx4Controller is a quadcopter control library implemented in C++ and provides a python interface through pybind11. ***The motivation of this work is to build a parallel flight control for large-scale quadrotor reinforcement learning (RL) and Sim-to-Real on the PX4 stack.*** Thus, the implementation of the controller is consistent with the Px4 flight control to ensure the consistency of sim2real.

## Features
rlPx4Controller is developed for DRL simulation flight control. The attributes include:
- CPU-based parallel flight geometric control computation.
- Strict **PX4-like** flight controller with differential flatness.
- Parallel control for different levels that supports: position & yaw control (**PY**), velocity & yaw control (**LV**), collective thrust & attitude angle (**CTA**), collective thrust & body rate (**CTBR**)

4 modules have been implemented so far:

- PolyTrajGen: Fifth degree polynomial trajectory generation
- Lemniscate:  Figure-eight curve generation
- pyControl: Single aircraft controller
- pyParallelControl：Multi-aircraft controller for reinforcement learning in vectorized environments.

For usage and more details, please refer to the [documentation](https://rlpx4controller.readthedocs.io/).