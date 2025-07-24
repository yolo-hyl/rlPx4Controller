"""

        Quadrotor Parallel Controller Utils
        -----------------------
    
"""
from __future__ import annotations
import numpy
import numpy.typing
import typing
__all__ = ['ParallelAttiControl', 'ParallelPosControl', 'ParallelRateControl', 'ParallelVelControl']
class ParallelAttiControl:
    def __init__(self, arg0: typing.SupportsInt) -> None:
        """
        b
        """
    def set_status(self, pos: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], q_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], vel: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], ang_vel: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], dt: typing.SupportsFloat) -> None:
        ...
    def update(self, actions: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"]:
        ...
class ParallelPosControl:
    def __init__(self, arg0: typing.SupportsInt) -> None:
        """
        b
        """
    def set_status(self, pos: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], q_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], vel: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], ang_vel: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], dt: typing.SupportsFloat) -> None:
        ...
    def update(self, actions: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"]:
        ...
class ParallelRateControl:
    def __init__(self, arg0: typing.SupportsInt) -> None:
        """
        batch calc
        """
    def set_q_world(self, q_world: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"]) -> None:
        ...
    def update(self, rate_sp: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], rate: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], dt: typing.SupportsFloat) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"]:
        ...
class ParallelVelControl:
    def __init__(self, arg0: typing.SupportsInt) -> None:
        """
        b
        """
    def set_status(self, pos: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], q_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], vel: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], ang_vel: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], dt: typing.SupportsFloat) -> None:
        ...
    def update(self, actions: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"]:
        ...
__version__: str = '0.0.2'
