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
import numpy as np
from dataclasses import dataclass, field
from typing import Tuple, Optional


@dataclass
class BodyPose:
    """
    Body orientation and position relative to world frame.
    
    Angles should be provided in radians.
    
    Rotation order: Roll (X) → Pitch (Y) → Yaw (Z)
    """
    roll: float = 0.0   # Rotation around X-axis (lateral tilt)
    pitch: float = 0.0  # Rotation around Y-axis (forward/backward tilt)
    yaw: float = 0.0    # Rotation around Z-axis (turning)
    x: float = 0.0      # Body center X offset
    y: float = 0.0      # Body center Y offset
    z: float = 0.0      # Body center Z offset
    
    def __post_init__(self):
        """Convert degrees to radians if needed"""
        # Allow degrees for convenience
        if abs(self.roll) > math.pi or abs(self.pitch) > math.pi or abs(self.yaw) > math.pi:
            # Likely in degrees, convert to radians
            self.roll = math.radians(self.roll)
            self.pitch = math.radians(self.pitch)
            self.yaw = math.radians(self.yaw)
    
    @classmethod
    def from_degrees(cls, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
                     x: float = 0.0, y: float = 0.0, z: float = 0.0) -> 'BodyPose':
        """Create BodyPose from degrees"""
        return cls(roll=roll, pitch=pitch, yaw=yaw, x=x, y=y, z=z)
    
    def get_rotation_matrix(self) -> np.ndarray:
        """
        Get 3x3 rotation matrix for this body pose.
        
        Uses Tait-Bryan angles with rotation order: R_z(yaw) * R_y(pitch) * R_x(roll)
        """
        cr, cp, cy = math.cos(self.roll), math.cos(self.pitch), math.cos(self.yaw)
        sr, sp, sy = math.sin(self.roll), math.sin(self.pitch), math.sin(self.yaw)
        
        # Rotation matrix elements
        # R = R_z * R_y * R_x
        r11 = cy * cp
        r12 = cy * sp * sr - sy * cr
        r13 = cy * sp * cr + sy * sr
        r21 = sy * cp
        r22 = sy * sp * sr + cy * cr
        r23 = sy * sp * cr - cy * sr
        r31 = -sp
        r32 = cp * sr
        r33 = cp * cr
        
        return np.array([
            [r11, r12, r13],
            [r21, r22, r23],
            [r31, r32, r33]
        ])
    
    def transform_point(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """
        Transform a point from body frame to world frame.
        
        Args:
            x, y, z: Point coordinates in body frame
            
        Returns:
            (x, y, z): Transformed coordinates in world frame
        """
        # Create homogeneous coordinates
        point = np.array([x, y, z, 1.0])
        
        # Get rotation matrix
        R = self.get_rotation_matrix()
        
        # Translation component
        t = np.array([self.x, self.y, self.z])
        
        # Apply transformation: [R | t] * [point; 1]
        transformed = R @ np.array([x, y, z]) + t
        
        return float(transformed[0]), float(transformed[1]), float(transformed[2])
    
    def inverse_transform_point(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """
        Transform a point from world frame to body frame.
        
        Args:
            x, y, z: Point coordinates in world frame
            
        Returns:
            (x, y, z): Transformed coordinates in body frame
        """
        # Apply inverse: R^T * (point - t)
        R = self.get_rotation_matrix()
        t = np.array([self.x, self.y, self.z])
        
        transformed = R.T @ (np.array([x, y, z]) - t)
        
        return float(transformed[0]), float(transformed[1]), float(transformed[2])


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
        
        y = (self.l2 * math.cos(angles.femur) * math.sin(angles.coxa) +
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
    
    def get_hip_position(self, leg_name: str) -> Tuple[float, float, float]:
        """
        Get the hip joint position in body coordinates for a given leg.
        
        Args:
            leg_name: Which leg (FL, FR, BL, BR)
            
        Returns:
            (x, y, z): Hip position relative to body center
        """
        if leg_name == "FL":
            return (-self.hip_offset_x, self.hip_offset_y, 0)
        elif leg_name == "FR":
            return (-self.hip_offset_x, -self.hip_offset_y, 0)
        elif leg_name == "BL":
            return (self.hip_offset_x, self.hip_offset_y, 0)
        elif leg_name == "BR":
            return (self.hip_offset_x, -self.hip_offset_y, 0)
        else:
            raise ValueError(f"Unknown leg: {leg_name}")
    
    def world_to_leg_angles(self, world_x: float, world_y: float, world_z: float,
                            leg_name: str, body_pose: Optional[BodyPose] = None) -> LegAngles:
        """
        Calculate joint angles for a foot position in world coordinates.
        
        This is the main integration point for body pose control.
        It transforms world coordinates to leg-local coordinates before
        calculating inverse kinematics.
        
        Args:
            world_x, world_y, world_z: Foot position in world frame
            leg_name: Which leg (FL, FR, BL, BR)
            body_pose: Current body pose (optional, default is neutral)
            
        Returns:
            LegAngles: Joint angles for the specified leg
        """
        if body_pose is None:
            body_pose = BodyPose()
        
        # Transform world coordinates to body frame
        body_x, body_y, body_z = body_pose.inverse_transform_point(world_x, world_y, world_z)
        
        # Convert body coordinates to leg-local coordinates
        leg_x, leg_y, leg_z = self.body_to_leg_coordinates(body_x, body_y, body_z, leg_name)
        
        # Calculate inverse kinematics
        return self.legs[leg_name].calculate_angles(leg_x, leg_y, leg_z)
    
    def calculate_all_legs_from_world(self, world_positions: dict, 
                                       body_pose: Optional[BodyPose] = None) -> dict:
        """
        Calculate angles for all legs given world coordinates and body pose.
        
        Args:
            world_positions: Dict mapping leg name to (x, y, z) in world frame
            body_pose: Current body pose (optional, default is neutral)
            
        Returns:
            Dict mapping leg name to LegAngles
        """
        if body_pose is None:
            body_pose = BodyPose()
        
        angles = {}
        for leg_name, world_pos in world_positions.items():
            angles[leg_name] = self.world_to_leg_angles(
                world_pos[0], world_pos[1], world_pos[2], leg_name, body_pose
            )
        return angles
    
    def apply_body_pose_to_stance(self, stance_width: float = 0.3, 
                                   stance_length: float = 0.4,
                                   stance_height: float = -0.3,
                                   body_pose: Optional[BodyPose] = None) -> dict:
        """
        Calculate foot positions for a neutral stance with body pose applied.
        
        This is useful for maintaining foot positions on the ground while
        tilting the body.
        
        Args:
            stance_width: Distance between left and right feet (meters)
            stance_length: Distance between front and back feet (meters)
            stance_height: Default foot height (meters)
            body_pose: Body pose to apply
            
        Returns:
            Dict mapping leg name to world coordinates
        """
        if body_pose is None:
            body_pose = BodyPose()
        
        # Neutral stance positions in body frame
        neutral_positions = {
            "FL": (-stance_length/2, stance_width/2, stance_height),
            "FR": (-stance_length/2, -stance_width/2, stance_height),
            "BL": (stance_length/2, stance_width/2, stance_height),
            "BR": (stance_length/2, -stance_width/2, stance_height)
        }
        
        # Transform each foot position from body frame to world frame
        world_positions = {}
        for leg_name, body_pos in neutral_positions.items():
            world_pos = body_pose.transform_point(*body_pos)
            world_positions[leg_name] = world_pos
        
        return world_positions
    
    def get_leg_ik_with_body_pose(self, leg_name: str, 
                                   body_relative_x: float,
                                   body_relative_y: float, 
                                   body_relative_z: float,
                                   body_pose: Optional[BodyPose] = None) -> LegAngles:
        """
        Convenience method: Calculate leg angles from body-relative coordinates.
        
        This applies the body pose transformation automatically.
        
        Args:
            leg_name: Which leg (FL, FR, BL, BR)
            body_relative_x: X position relative to body center
            body_relative_y: Y position relative to body center
            body_relative_z: Z position relative to body center
            body_pose: Current body pose
            
        Returns:
            LegAngles: Joint angles for the leg
        """
        if body_pose is None:
            body_pose = BodyPose()
        
        # Transform body coordinates to world frame
        world_x, world_y, world_z = body_pose.transform_point(
            body_relative_x, body_relative_y, body_relative_z
        )
        
        # Convert to leg-local coordinates
        leg_x, leg_y, leg_z = self.body_to_leg_coordinates(
            body_relative_x, body_relative_y, body_relative_z, leg_name
        )
        
        # Calculate IK
        return self.legs[leg_name].calculate_angles(leg_x, leg_y, leg_z)

