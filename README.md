# rlPx4Controller

rlPx4Controller is a quadcopter control library implemented in C++ and provides a python interface through pybind11. The implementation of the controller is consistent with the Px4 flight control to ensure the consistency of sim2real.

4 modules have been implemented so far

- PolyTrajGen: Fifth degree polynomial trajectory generation
- Lemniscate:  Figure-eight curve generation
- pyControl: Single aircraft controller
- pyParallelControl：Multi-aircraft controller for reinforcement learning

For usage and more details, please refer to the [documentation](https://rlpx4controller.readthedocs.io/).