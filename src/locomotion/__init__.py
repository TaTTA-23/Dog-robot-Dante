"""Módulo de locomoção do Dante."""

from .locomotion import Locomotion
from .kinematics import (
    QuadrupedKinematics,
    LegKinematics,
    LegAngles,
    CartesianPosition,
    BodyPose
)
from .gait import GaitController, GaitConfig, GaitType, TrajectoryGenerator

try:
    from .simulation_wrapper import PyBulletWrapper, RobotConfig
    SIMULATION_AVAILABLE = True
except ImportError:
    SIMULATION_AVAILABLE = False
    PyBulletWrapper = None
    RobotConfig = None

__all__ = [
    "Locomotion",
    "QuadrupedKinematics",
    "LegKinematics", 
    "LegAngles",
    "CartesianPosition",
    "BodyPose",
    "GaitController",
    "GaitConfig",
    "GaitType",
    "TrajectoryGenerator",
    "PyBulletWrapper",
    "RobotConfig",
    "SIMULATION_AVAILABLE"
]
