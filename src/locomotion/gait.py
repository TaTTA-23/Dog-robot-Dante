"""
Gait Module for Dante Quadruped Robot

This module implements the trajectory generation for quadruped locomotion.
Each leg follows an elliptical path during the swing phase of gait.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple
from enum import Enum

from .kinematics import QuadrupedKinematics, LegAngles


class GaitType(Enum):
    """Supported gait types"""
    WALK = "walk"
    TROT = "trot"
    PACE = "pace"
    GALLOP = "gallop"


@dataclass
class GaitConfig:
    """Configuration parameters for gait generation"""
    step_length: float = 0.08
    step_height: float = 0.05
    step_width: float = 0.12
    stance_ratio: float = 0.6
    cycle_period: float = 0.5
    body_height: float = 0.15
    
    def __post_init__(self):
        if not 0.0 < self.stance_ratio < 1.0:
            raise ValueError(f"stance_ratio must be between 0 and 1")


class TrajectoryGenerator:
    """Generates foot trajectories using elliptical paths"""
    
    def __init__(self, config: GaitConfig = None):
        self.config = config or GaitConfig()
    
    def generate_elliptical_trajectory(self,
                                        start_pos: Tuple[float, float, float],
                                        end_pos: Tuple[float, float, float],
                                        phase: float) -> Tuple[float, float, float]:
        """
        Generate position using elliptical trajectory.
        
        Args:
            start_pos: Starting position
            end_pos: Ending position
            phase: Progress through swing (0.0 to 1.0)
        """
        # Cubic smoothstep for natural acceleration
        smooth_phase = phase * phase * (3 - 2 * phase)
        
        # Linear interpolation for horizontal movement
        x = start_pos[0] + smooth_phase * (end_pos[0] - start_pos[0])
        y = start_pos[1] + smooth_phase * (end_pos[1] - start_pos[1])
        
        # Elliptical vertical profile
        swing_height = self.config.step_height
        z = start_pos[2] + swing_height * math.sin(phase * math.pi)
        
        return (x, y, z)


class GaitController:
    """High-level gait controller that coordinates all 4 legs"""
    
    def __init__(self, kinematics: QuadrupedKinematics, config: GaitConfig = None):
        self.kinematics = kinematics
        self.config = config or GaitConfig()
        self.trajectory = TrajectoryGenerator(self.config)
        
        # Default neutral positions
        self._default_neutral = {
            "FL": (0.15, 0.08, -0.15),
            "FR": (0.15, -0.08, -0.15),
            "BL": (-0.15, 0.08, -0.15),
            "BR": (-0.15, -0.08, -0.15),
        }
        
        self._gait_phase = 0.0
        
        # Phase offsets for gait patterns
        self._leg_phases = {"FL": 0.0, "FR": 0.5, "BL": 0.5, "BR": 0.0}
    
    def update(self, dt: float, velocity: Tuple[float, float, float] = (0, 0, 0)) -> Dict[str, LegAngles]:
        """Update gait state and calculate target angles"""
        phase_advance = dt / self.config.cycle_period
        self._gait_phase = (self._gait_phase + phase_advance) % 1.0
        
        target_angles = {}
        for leg_name in self.kinematics.legs.keys():
            leg_phase = (self._gait_phase + self._leg_phases[leg_name]) % 1.0
            
            if leg_phase < self.config.stance_ratio:
                target_pos = self._calculate_stance_position(leg_name, velocity)
            else:
                swing_phase = (leg_phase - self.config.stance_ratio) / (1 - self.config.stance_ratio)
                target_pos = self._calculate_swing_position(leg_name, swing_phase)
            
            leg_local = self.kinematics.body_to_leg_coordinates(
                target_pos[0], target_pos[1], target_pos[2], leg_name
            )
            
            try:
                angles = self.kinematics.legs[leg_name].calculate_angles(*leg_local)
                target_angles[leg_name] = angles
            except ValueError as e:
                print(f"Warning: IK failed for {leg_name}: {e}")
                target_angles[leg_name] = None
        
        return target_angles
    
    def _calculate_stance_position(self, leg_name: str, velocity: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Calculate foot position during stance phase"""
        vx, vy, _ = velocity
        neutral = self._default_neutral[leg_name]
        
        stance_offset_x = -vx * self.config.cycle_period * self.config.stance_ratio
        stance_offset_y = -vy * self.config.cycle_period * self.config.stance_ratio
        
        return (neutral[0] + stance_offset_x, neutral[1] + stance_offset_y, neutral[2])
    
    def _calculate_swing_position(self, leg_name: str, swing_phase: float) -> Tuple[float, float, float]:
        """Calculate foot position during swing phase"""
        neutral = self._default_neutral[leg_name]
        swing_target = (neutral[0] + self.config.step_length, neutral[1], neutral[2])
        
        return self.trajectory.generate_elliptical_trajectory(neutral, swing_target, swing_phase)
    
    def set_gait_type(self, gait_type: GaitType):
        """Change the gait type"""
        if gait_type == GaitType.WALK:
            self._leg_phases = {"FL": 0.0, "BL": 0.25, "FR": 0.5, "BR": 0.75}
        elif gait_type == GaitType.TROT:
            self._leg_phases = {"FL": 0.0, "BR": 0.0, "BL": 0.5, "FR": 0.5}
        elif gait_type == GaitType.PACE:
            self._leg_phases = {"FL": 0.0, "FR": 0.0, "BL": 0.5, "BR": 0.5}
        elif gait_type == GaitType.GALLOP:
            self._leg_phases = {"FL": 0.0, "FR": 0.1, "BL": 0.5, "BR": 0.6}
        
        print(f"Switched to {gait_type.value} gait")


class LocomotionControl:
    """Main locomotion controller integrating kinematics and gait"""
    
    def __init__(self):
        self.kinematics = QuadrupedKinematics(
            coxa_length=0.05,
            femur_length=0.10,
            tibia_length=0.15
        )
        self.gait = GaitController(self.kinematics)
        self._current_commands = {}
        print("Locomotion control system initialized")
    
    def walk(self, velocity: Tuple[float, float, float] = (0.1, 0, 0)) -> str:
        """Execute walking motion"""
        dt = 0.02
        self.gait.update(dt, velocity)
        return "walking"
    
    def run(self) -> str:
        """Execute running motion"""
        self.gait.set_gait_type(GaitType.TROT)
        self.gait.config.step_length = 0.15
        self.gait.config.cycle_period = 0.3
        return "running"
    
    def jump(self) -> str:
        """Execute jumping motion"""
        return "jumped"
    
    def climb(self) -> str:
        """Execute climbing motion"""
        return "climbed"
    
    def move_servos(self, angles: Dict[str, LegAngles]):
        """Send angle commands to motors"""
        self._current_commands = angles

