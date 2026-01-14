"""
Kinematics Module for Dante Quadruped Robot

This module implements the Inverse Kinematics (IK) solver for the 3-DOF legs.
Each leg has:
- Coxa (Hip/Abduction): Movement in/out
- Fêmur (Upper Leg): Movement forward/backward  
- Tíbia (Lower Leg): Extension/contraction

Coordinate System:
- Origin (0,0,0) at the shoulder joint
- X: Forward direction (robot's forward)
- Y: Lateral direction (abduction/adduction)
- Z: Vertical direction (up/down)
"""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class LegAngles:
    """Joint angles for a single leg (in radians)"""
    coxa: float   # Hip angle (theta1)
    femur: float  # Upper leg angle (theta2)
    tibia: float  # Lower leg angle (theta3)


@dataclass
class CartesianPosition:
    """Cartesian position relative to shoulder joint"""
    x: float  # Forward (meters)
    y: float  # Lateral (meters)
    z: float  # Vertical (meters)


class LegKinematics:
    """
    Inverse Kinematics solver for a 3-DOF leg.
    
    Uses the Law of Cosines to calculate joint angles from
    target Cartesian coordinates.
    """
    
    def __init__(self, l1: float, l2: float, l3: float, leg_name: str = "leg"):
        """
        Initialize leg kinematics with link lengths.
        
        Args:
            l1: Coxa length (hip joint to femur joint) [meters]
            l2: Femur length (femur joint to knee joint) [meters]  
            l3: Tibia length (knee joint to foot) [meters]
            leg_name: Identifier for this leg (e.g., "FL", "FR", "BL", "BR")
        """
        self.l1 = l1  # Hip link
        self.l2 = l2  # Upper leg link
        self.l3 = l3  # Lower leg link
        self.leg_name = leg_name
        
        # Total leg reach
        self.max_reach = l1 + l2 + l3
        
        # Pre-calculate some values for efficiency
        self.l2_sq = l2 ** 2
        self.l3_sq = l3 ** 2
        self.l2_l3_2 = 2 * l2 * l3
        
        print(f"Initialized {leg_name} kinematics: l1={l1}m, l2={l2}m, l3={l3}m")
    
    def calculate_angles(self, x: float, y: float, z: float) -> LegAngles:
        """
        Calculate joint angles for a target Cartesian position.
        
        Args:
            x: Forward distance from shoulder [meters]
            y: Lateral distance from shoulder [meters]
            z: Vertical distance from shoulder [meters]
            
        Returns:
            LegAngles: Joint angles (coxa, femur, tibia) in radians
            
        Raises:
            ValueError: If target position is out of reach
        """
        # Calculate horizontal distance from shoulder axis
        # This is the distance in the Y-Z plane (perpendicular to X)
        horizontal_dist = math.sqrt(y**2 + z**2)
        
        # Check if target is reachable
        if horizontal_dist < self.l1 - 0.001:
            # Target is too close to the body (inside minimum reach)
            raise ValueError(
                f"{self.leg_name}: Target too close to body. "
                f"Min horizontal distance: {self.l1}m"
            )
        
        # Distance from coxa joint to target point in 3D
        # This is the hypotenuse in the X-horizontal_dist plane
        d = math.sqrt(x**2 + horizontal_dist**2)
        
        if d > self.l2 + self.l3 - 0.001:
            # Target is out of reach
            raise ValueError(
                f"{self.leg_name}: Target out of reach. "
                f"Max reach: {self.l2 + self.l3}m, requested: {d}m"
            )
        
        # ========== Theta 1 (Coxa / Hip Angle) ==========
        # atan2 gives correct quadrant handling
        # Angle in Y-Z plane relative to the body plane
        theta1 = math.atan2(y, z)
        
        # ========== Calculate intermediate variables ==========
        # Distance from femur joint to target point
        # Using Pythagorean theorem: d^2 = x^2 + (horizontal_dist - l1)^2
        d_prime = math.sqrt(x**2 + (horizontal_dist - self.l1)**2)
        
        # ========== Theta 2 (Femur Angle) ==========
        # Use Law of Cosines to find the angle at the femur joint
        # cos(gamma) = (l2^2 + d'^2 - l3^2) / (2 * l2 * d')
        # This gives the angle between femur and the line to target
        cos_gamma = (self.l2_sq + d_prime**2 - self.l3_sq) / (2 * self.l2 * d_prime)
        
        # Clamp to valid range to avoid numerical errors
        cos_gamma = max(-1.0, min(1.0, cos_gamma))
        gamma = math.acos(cos_gamma)
        
        # Angle of the target vector relative to horizontal
        alpha = math.atan2(x, horizontal_dist - self.l1)
        
        # Femur angle is alpha + gamma (measured from vertical)
        theta2 = alpha + gamma
        
        # ========== Theta 3 (Tibia Angle) ==========
        # Use Law of Cosines for the knee joint angle
        # cos(delta) = (l2^2 + l3^2 - d'^2) / (2 * l2 * l3)
        cos_delta = (self.l2_sq + self.l3_sq - d_prime**2) / self.l2_l3_2
        
        # Clamp to valid range
        cos_delta = max(-1.0, min(1.0, cos_delta))
        delta = math.acos(cos_delta)
        
        # Tibia angle is pi - delta (relative to femur extension)
        # Negative because knee bends "backward" for dog legs
        theta3 = delta - math.pi
        
        return LegAngles(coxa=theta1, femur=theta2, tibia=theta3)
    
    def calculate_position(self, angles: LegAngles) -> CartesianPosition:
        """
        Forward Kinematics: Calculate Cartesian position from joint angles.
        Useful for validation and testing.
        
        Args:
            angles: Joint angles (coxa, femur, tibia) in radians
            
        Returns:
            CartesianPosition: (x, y, z) coordinates
        """
        x = (self.l2 * math.sin(angles.femur) + 
             self.l3 * math.sin(angles.femur + angles.tibia))
        
        y = (self.l1 + 
             self.l2 * math.cos(angles.femur) * math.sin(angles.coxa) +
             self.l3 * math.cos(angles.femur + angles.tibia) * math.sin(angles.coxa))
        
        z = (self.l2 * math.cos(angles.femur) * math.cos(angles.coxa) +
             self.l3 * math.cos(angles.femur + angles.tibia) * math.cos(angles.coxa))
        
        return CartesianPosition(x=x, y=y, z=z)
    
    def get_ik_range(self) -> dict:
        """Return the reachable workspace bounds"""
        min_reach = abs(self.l2 - self.l3)
        max_reach = self.l2 + self.l3
        
        return {
            "min_horizontal": self.l1,
            "min_total": math.sqrt(self.l1**2 + min_reach**2),
            "max_total": math.sqrt(self.l1**2 + max_reach**2),
            "x_range": (0, max_reach),  # Forward
            "y_range": (-self.l1, self.l1),  # Lateral from body
            "z_range": (-max_reach, max_reach)  # Vertical
        }


class QuadrupedKinematics:
    """
    Manages kinematics for all 4 legs of the quadruped.
    
    Leg naming convention:
    - FL: Front Left
    - FR: Front Right  
    - BL: Back Left
    - BR: Back Right
    """
    
    def __init__(self, 
                 coxa_length: float = 0.05,    # 50mm
                 femur_length: float = 0.10,   # 100mm
                 tibia_length: float = 0.15,   # 150mm
                 hip_offset_x: float = 0.15,   # Distance from center to front hips
                 hip_offset_y: float = 0.08):  # Distance from center to side hips
        """
        Initialize all 4 legs with geometry.
        
        Args:
            coxa_length: Length of hip link [meters]
            femur_length: Length of upper leg [meters]
            tibia_length: Length of lower leg [meters]
            hip_offset_x: X distance from body center to hip
            hip_offset_y: Y distance from body center to hip
        """
        self.hip_offset_x = hip_offset_x
        self.hip_offset_y = hip_offset_y
        
        # Create legs with appropriate offsets
        self.legs = {
            "FL": LegKinematics(coxa_length, femur_length, tibia_length, "Front Left"),
            "FR": LegKinematics(coxa_length, femur_length, tibia_length, "Front Right"),
            "BL": LegKinematics(coxa_length, femur_length, tibia_length, "Back Left"),
            "BR": LegKinematics(coxa_length, femur_length, tibia_length, "Back Right"),
        }
        
        print(f"Quadruped kinematics initialized with {len(self.legs)} legs")
    
    def calculate_all_angles(self, positions: dict) -> dict:
        """
        Calculate angles for all legs given target positions.
        
        Args:
            positions: Dict mapping leg name to (x, y, z) tuple
            
        Returns:
            Dict mapping leg name to LegAngles
        """
        angles = {}
        for leg_name, pos in positions.items():
            if leg_name in self.legs:
                angles[leg_name] = self.legs[leg_name].calculate_angles(*pos)
            else:
                raise ValueError(f"Unknown leg: {leg_name}")
        return angles
    
    def body_to_leg_coordinates(self, body_x: float, body_y: float, body_z: float,
                                 leg_name: str) -> Tuple[float, float, float]:
        """
        Convert body-relative coordinates to leg-relative coordinates.
        
        The body coordinate system has origin at the robot's center.
        Leg coordinate systems have origin at each hip joint.
        
        Args:
            body_x: X position in body frame (forward from center)
            body_y: Y position in body frame (lateral from center)
            body_z: Z position in body frame (up from center)
            leg_name: Which leg to calculate for
            
        Returns:
            (x, y, z) in leg's local coordinate system
        """
        # Apply hip offset based on leg position
        if leg_name == "FL":
            return (body_x - self.hip_offset_x, 
                    body_y + self.hip_offset_y, 
                    body_z)
        elif leg_name == "FR":
            return (body_x - self.hip_offset_x, 
                    body_y - self.hip_offset_y, 
                    body_z)
        elif leg_name == "BL":
            return (body_x + self.hip_offset_x, 
                    body_y + self.hip_offset_y, 
                    body_z)
        elif leg_name == "BR":
            return (body_x + self.hip_offset_x, 
                    body_y - self.hip_offset_y, 
                    body_z)
        else:
            raise ValueError(f"Unknown leg: {leg_name}")
    
    def leg_to_body_coordinates(self, leg_x: float, leg_y: float, leg_z: float,
                                 leg_name: str) -> Tuple[float, float, float]:
        """
        Convert leg-local coordinates to body-relative coordinates.
        
        Args:
            leg_x: X position in leg's local frame
            leg_y: Y position in leg's local frame
            leg_z: Z position in leg's local frame
            leg_name: Which leg
            
        Returns:
            (x, y, z) in body coordinate system
        """
        if leg_name == "FL":
            return (leg_x + self.hip_offset_x, 
                    leg_y - self.hip_offset_y, 
                    leg_z)
        elif leg_name == "FR":
            return (leg_x + self.hip_offset_x, 
                    leg_y + self.hip_offset_y, 
                    leg_z)
        elif leg_name == "BL":
            return (leg_x - self.hip_offset_x, 
                    leg_y - self.hip_offset_y, 
                    leg_z)
        elif leg_name == "BR":
            return (leg_x - self.hip_offset_x, 
                    leg_y + self.hip_offset_y, 
                    leg_z)
        else:
            raise ValueError(f"Unknown leg: {leg_name}")

