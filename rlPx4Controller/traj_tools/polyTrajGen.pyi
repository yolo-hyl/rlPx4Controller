"""

        polynomial_traj
        -----------------------
    
"""
from __future__ import annotations
import numpy
import numpy.typing
import typing
__all__ = ['PolyTrajGen']
class PolyTrajGen:
    def __init__(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"], arg2: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], arg3: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], arg4: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], arg5: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> None:
        ...
    def reset_traj(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"], arg2: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], arg3: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], arg4: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], arg5: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> None:
        ...
    def sample(self, arg0: typing.SupportsFloat) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
        ...
    def sample_acc(self, arg0: typing.SupportsFloat) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
        ...
    def sample_vel(self, arg0: typing.SupportsFloat) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
        ...
__version__: str = '0.0.2'
