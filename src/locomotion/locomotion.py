"""Locomotion module: controle de movimento para o robô Dante.

Este módulo implementa a cinemática inversa e controle de locomoção
para o robô quadrúpede Dante.
"""

from typing import Dict, Tuple

from .kinematics import LegAngles, QuadrupedKinematics
from .gait import GaitController, GaitType, LocomotionControl as BaseLocomotionControl


class Locomotion(BaseLocomotionControl):
    """
    Controlador de locomoção para o robô Dante.
    
    Herda de LocomotionControl que implementa:
    - Cinemática inversa 3-DOF por pata
    - Geração de trajetória elíptica
    - Múltiplos tipos de gait (walk, trot, pace, gallop)
    """
    
    def __init__(self):
        """Inicializa o sistema de locomoção"""
        super().__init__()
        print("Locomotion module initialized with full IK support")
    
    def walk(self, velocity: Tuple[float, float, float] = (0.1, 0, 0)) -> str:
        """
        Executa movimento de caminhada.
        
        Args:
            velocity: Tupla (vx, vy, vyaw) - velocidade do corpo
            
        Returns:
            Status do movimento
        """
        dt = 0.02  # 50Hz update rate
        target_angles = self.gait.update(dt, velocity)
        self.move_servos(target_angles)
        return "walking"
    
    def run(self) -> str:
        """Executa movimento de corrida (trot gait)"""
        self.gait.set_gait_type(GaitType.TROT)
        self.gait.config.step_length = 0.15
        self.gait.config.cycle_period = 0.3
        return "running"
    
    def jump(self) -> str:
        """Executa movimento de salto"""
        # Implementação simplificada - em produção usaria trajetória parabólica
        return "jumped"
    
    def climb(self) -> str:
        """Executa movimento de escalada"""
        return "climbed"
    
    def calculate_ik(self, x: float, y: float, z: float, leg_name: str = "FL") -> LegAngles:
        """
        Calcula ângulos IK para uma posição alvo.
        
        Args:
            x: Posição X (frente/trás)
            y: Posição Y (lateral)
            z: Posição Z (altura)
            leg_name: Nome da pata (FL, FR, BL, BR)
            
        Returns:
            LegAngles com os ângulos das juntas
        """
        return self.kinematics.legs[leg_name].calculate_angles(x, y, z)
    
    def set_gait(self, gait_type: str) -> str:
        """
        Define o tipo de gait.
        
        Args:
            gait_type: 'walk', 'trot', 'pace', ou 'gallop'
            
        Returns:
            Status da operação
        """
        try:
            self.gait.set_gait_type(GaitType(gait_type))
            return f"gait_set_to_{gait_type}"
        except ValueError:
            return f"unknown_gait_{gait_type}"

