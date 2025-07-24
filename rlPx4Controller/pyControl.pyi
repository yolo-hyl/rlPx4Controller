"""

        Quadrotor Controller Utils
        -----------------------
    
"""
from __future__ import annotations
import numpy
import numpy.typing
import typing
__all__ = ['AttiControl', 'Mixer', 'PosControl', 'RateControl']
class AttiControl:
    def __init__(self) -> None:
        """
        Quaternion nonlinear attitude control, output in body coordinates
        """
    def set_pid_params(self, p_gains: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> None:
        ...
    def update(self, q_sp: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"], q: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
        ...
class Mixer:
    def __init__(self) -> None:
        ...
    def update(self, torque: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[4, 1]"]:
        ...
class PosControl:
    def __init__(self) -> None:
        ...
    def get_hover_thrust(self) -> float:
        ...
    def set_pid_params(self, pos_gains: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], vel_gains: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> None:
        ...
    def set_status(self, pos: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], vel: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], angular_velocity: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], q: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"], dt: typing.SupportsFloat) -> None:
        ...
    def update(self, pos_sp: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], vel_sp: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], acc_sp: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], yaw_sp: typing.SupportsFloat) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        ...
class RateControl:
    def __init__(self) -> None:
        ...
    def set_pid_params(self, p_gains: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], i_gains: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], d_gains: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> None:
        ...
    def set_q_world(self, q_world: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"]) -> None:
        ...
    def update(self, rate_sp: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], rate: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], angular_accel: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], dt: typing.SupportsFloat) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
        ...
__version__: str = '0.0.2'
