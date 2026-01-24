"""
PyBullet Simulation Wrapper for Dante Quadruped Robot

This module provides an interface between the locomotion controller
and a PyBullet physics simulation. It creates a 3D robot model and
applies joint angles from the kinematics module.

Benefits:
- Test gait stability with real physics (gravity, inertia, collisions)
- Debug IK solutions before deploying to hardware
- Visualize robot motion in 3D
- Experiment with different gaits safely

Usage:
    from simulation_wrapper import PyBulletWrapper
    
    sim = PyBulletWrapper()
    sim.create_robot()
    
    # In the control loop:
    angles = gait.update(dt, velocity)
    sim.set_joint_angles(angles)
    sim.step()
"""

import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, List
from contextlib import contextmanager

# PyBullet imports - will be None if not installed
try:
    import pybullet as pb
    import pybullet_data
    PYBULLET_AVAILABLE = True
except ImportError:
    PYBULLET_AVAILABLE = False
    pb = None

from .kinematics import QuadrupedKinematics, LegAngles, BodyPose


@dataclass
class RobotConfig:
    """Physical configuration of the robot for simulation"""
    # Body dimensions
    body_width: float = 0.15
    body_length: float = 0.25
    body_height: float = 0.08
    
    # Leg link lengths (must match kinematics)
    coxa_length: float = 0.05
    femur_length: float = 0.10
    tibia_length: float = 0.15
    
    # Hip offsets from body center
    hip_offset_x: float = 0.15
    hip_offset_y: float = 0.08
    
    # Joint damping for realistic physics
    joint_damping: float = 0.1
    
    # Maximum joint torque
    max_torque: float = 5.0
    
    # Mass properties
    body_mass: float = 2.0
    leg_mass: float = 0.2


class JointInfo:
    """Information about a robot joint"""
    def __init__(self, name: str, joint_index: int, leg_name: str, joint_type: str):
        self.name = name
        self.joint_index = joint_index
        self.leg_name = leg_name
        self.joint_type = joint_type  # 'coxa', 'femur', 'tibia'
        self.current_angle = 0.0
        self.target_angle = 0.0


class PyBulletWrapper:
    """
    Wrapper for PyBullet simulation.
    
    Creates a simple 3D representation of Dante and allows controlling
    it with the same joint angles produced by the kinematics module.
    """
    
    def __init__(self, config: RobotConfig = None, gui: bool = True):
        """
        Initialize the simulation wrapper.
        
        Args:
            config: Robot configuration (default values provided)
            gui: Whether to show PyBullet GUI (default True)
        """
        if not PYBULLET_AVAILABLE:
            raise ImportError(
                "PyBullet is not installed. Install with: pip install pybullet"
            )
        
        self.config = config or RobotConfig()
        self.gui = gui
        self.robot_id = None
        self.plane_id = None
        self.joints: Dict[str, JointInfo] = {}
        self.is_connected = False
        self.simulation_time = 0.0
        
        # Store kinematics for transformations
        self.kinematics = QuadrupedKinematics(
            coxa_length=self.config.coxa_length,
            femur_length=self.config.femur_length,
            tibia_length=self.config.tibia_length,
            hip_offset_x=self.config.hip_offset_x,
            hip_offset_y=self.config.hip_offset_y
        )
    
    def connect(self):
        """Connect to PyBullet physics server"""
        if self.gui:
            self.client_id = pb.connect(pb.GUI)
        else:
            self.client_id = pb.connect(pb.DIRECT)
        
        # Set gravity
        pb.setGravity(0, 0, -9.81)
        
        # Add search path for URDF files
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Load plane
        self.plane_id = pb.loadURDF("plane.urdf")
        
        # Configure physics
        pb.setPhysicsEngineParameter(
            fixedTimeStep=1.0/240.0,
            numSolverIterations=10,
            numSubSteps=4
        )
        
        self.is_connected = True
        print(f"PyBullet connected (client {self.client_id})")
    
    def create_robot(self, start_position: Tuple[float, float, float] = (0, 0, 0.4)):
        """
        Create the robot model in the simulation.
        
        Uses primitive shapes (boxes, cylinders) to represent the robot
        since we don't have a URDF file yet.
        """
        if not self.is_connected:
            raise RuntimeError("Not connected to PyBullet. Call connect() first.")
        
        # Create compound body from multiple shapes
        # Using createMultiBody to create a body with attached links
        
        # Base body (box)
        body_collision = pb.createCollisionShape(
            pb.GEOM_BOX,
            halfExtents=[
                self.config.body_length / 2,
                self.config.body_width / 2,
                self.config.body_height / 2
            ]
        )
        
        body_visual = pb.createVisualShape(
            pb.GEOM_BOX,
            halfExtents=[
                self.config.body_length / 2,
                self.config.body_width / 2,
                self.config.body_height / 2
            ],
            rgbaColor=[0.3, 0.5, 0.8, 1.0]  # Blue-ish
        )
        
        # Create base inertia
        body_mass = self.config.body_mass
        body_inertia = pb.calculateInertia(
            body_mass,
            halfExtents=[
                self.config.body_length / 2,
                self.config.body_width / 2,
                self.config.body_height / 2
            ]
        )
        
        # Create multi-body robot
        self.robot_id = pb.createMultiBody(
            baseMass=body_mass,
            baseCollisionShapeIndex=body_collision,
            baseVisualShapeIndex=body_visual,
            basePosition=start_position,
            baseInertialFrameOrientation=[0, 0, 0, 1]  # Identity quaternion
        )
        
        # Add legs with joints
        self._create_legs()
        
        print(f"Robot created with ID {self.robot_id}")
        return self.robot_id
    
    def _create_legs(self):
        """Create all 4 legs with joints"""
        leg_positions = {
            "FL": (-self.config.hip_offset_x, self.config.hip_offset_y, 0),
            "FR": (-self.config.hip_offset_x, -self.config.hip_offset_y, 0),
            "BL": (self.config.hip_offset_x, self.config.hip_offset_y, 0),
            "BR": (self.config.hip_offset_x, -self.config.hip_offset_y, 0),
        }
        
        # Y-direction multiplier for right legs
        right_legs = {"FR", "BR"}
        
        for leg_name, (lx, ly, lz) in leg_positions.items():
            is_right = leg_name in right_legs
            y_multiplier = -1 if is_right else 1
            
            # Create coxa link
            coxa_parent = self.robot_id
            coxa_position = [lx, ly, lz]
            
            # Coxa joint (hip abduction/adduction)
            coxa_joint_index = self._create_link(
                parent=coxa_parent,
                name=f"{leg_name}_coxa_joint",
                leg_name=leg_name,
                joint_type="coxa",
                length=self.config.coxa_length,
                parent_position=coxa_position,
                joint_axis=[0, 1, 0],  # Y-axis rotation
                mass=0.05,
                color=[0.6, 0.3, 0.1, 1.0]  # Brown
            )
            
            # Coxa visual link (for visualization)
            coxa_link = self._create_visual_link(
                parent=coxa_joint_index,
                length=self.config.coxa_length,
                position=[0, 0, 0],
                color=[0.6, 0.3, 0.1, 1.0]
            )
            
            # Femur joint
            femur_parent = coxa_joint_index
            femur_position = [self.config.coxa_length, 0, 0]
            
            femur_joint_index = self._create_link(
                parent=femur_parent,
                name=f"{leg_name}_femur_joint",
                leg_name=leg_name,
                joint_type="femur",
                length=self.config.femur_length,
                parent_position=femur_position,
                joint_axis=[0, 0, 1],  # Z-axis rotation
                mass=0.1,
                color=[0.5, 0.5, 0.5, 1.0]  # Gray
            )
            
            # Femur visual link
            femur_link = self._create_visual_link(
                parent=femur_joint_index,
                length=self.config.femur_length,
                position=[self.config.femur_length / 2, 0, 0],
                color=[0.5, 0.5, 0.5, 1.0]
            )
            
            # Tibia joint (knee)
            tibia_parent = femur_joint_index
            tibia_position = [self.config.femur_length, 0, 0]
            
            tibia_joint_index = self._create_link(
                parent=tibia_parent,
                name=f"{leg_name}_tibia_joint",
                leg_name=leg_name,
                joint_type="tibia",
                length=self.config.tibia_length,
                parent_position=tibia_position,
                joint_axis=[0, 0, 1],  # Z-axis rotation
                mass=0.15,
                color=[0.4, 0.4, 0.4, 1.0]  # Dark gray
            )
            
            # Tibia visual link (with foot at end)
            tibia_link = self._create_visual_link(
                parent=tibia_joint_index,
                length=self.config.tibia_length,
                position=[self.config.tibia_length / 2, 0, 0],
                color=[0.4, 0.4, 0.4, 1.0]
            )
            
            # Add foot sphere at end of tibia
            self._create_foot(tibia_joint_index, leg_name)
    
    def _create_link(self, parent: int, name: str, leg_name: str,
                     joint_type: str, length: float, parent_position: List[float],
                     joint_axis: List[float], mass: float, color: List[float]) -> int:
        """
        Create a link with a joint.
        
        Returns the body index of the created link.
        """
        # Create collision shape
        collision = pb.createCollisionShape(
            pb.GEOM_CYLINDER,
            radius=0.015,
            height=length,
            collisionFramePosition=[length / 2, 0, 0],
            collisionFrameOrientation=[0, 0.7071, 0, 0.7071]  # Rotate 90 deg around Y
        )
        
        # Create visual shape
        visual = pb.createVisualShape(
            pb.GEOM_CYLINDER,
            radius=0.018,
            height=length,
            visualFramePosition=[length / 2, 0, 0],
            visualFrameOrientation=[0, 0.7071, 0, 0.7071],
            rgbaColor=color
        )
        
        # Create link with joint
        link_mass = mass
        link_inertia = pb.calculateInertia(
            link_mass,
            halfExtents=[length/2, 0.02, 0.02]
        )
        
        # Use createBody to make the link
        # The parent joint is created separately
        body_index = pb.createMultiBody(
            baseMass=link_mass,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=parent_position,
            baseInertialFrameOrientation=[0, 0, 0, 1],
            linkCollisionShapes=[],
            linkVisualShapes=[],
            linkMasses=[],
            linkPositions=[],
            linkOrientations=[],
            linkInertialFrameOrientation=[],
            parentObjectIndex=parent,
            parentLinkIndex=-1  # Base link
        )
        
        # Create the joint connecting to parent
        joint_type_id = pb.JOINT_REVOLUTE if joint_type != "fixed" else pb.JOINT_FIXED
        
        joint_index = pb.createJoint(
            bodyIndex=body_index,
            parentBodyIndex=parent,
            jointType=joint_type_id,
            jointAxis=joint_axis,
            parentFramePosition=parent_position,
            childFramePosition=[0, 0, 0],
            childFrameOrientation=[0, 0, 0, 1]
        )
        
        # Configure joint
        pb.setJointMotorControl2(
            bodyIndex=self.robot_id,
            jointIndex=joint_index,
            controlMode=pb.POSITION_CONTROL,
            targetPosition=0,
            force=self.config.max_torque,
            positionGain=0.5,
            velocityGain=0.5
        )
        
        # Enable joint damping
        pb.setJointDynamics(
            bodyIndex=self.robot_id,
            jointIndex=joint_index,
            linearDamping=self.config.joint_damping,
            angularDamping=self.config.joint_damping
        )
        
        # Store joint info
        self.joints[name] = JointInfo(
            name=name,
            joint_index=joint_index,
            leg_name=leg_name,
            joint_type=joint_type
        )
        
        return body_index
    
    def _create_visual_link(self, parent: int, length: float, 
                            position: List[float], color: List[float]):
        """Create a visual-only link (no physics)"""
        visual = pb.createVisualShape(
            pb.GEOM_CYLINDER,
            radius=0.02,
            height=length,
            visualFramePosition=[length/2, 0, 0],
            visualFrameOrientation=[0, 0.7071, 0, 0.7071],
            rgbaColor=color
        )
        
        link_mass = 0.0  # Massless (visual only)
        link_inertia = [0.001, 0.001, 0.001]
        
        body_index = pb.createMultiBody(
            baseMass=link_mass,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=visual,
            basePosition=position,
            baseInertialFrameOrientation=[0, 0, 0, 1],
            linkCollisionShapes=[],
            linkVisualShapes=[],
            linkMasses=[],
            linkPositions=[],
            linkOrientations=[],
            linkInertialFrameOrientation=[],
            parentObjectIndex=parent,
            parentLinkIndex=-1
        )
        
        return body_index
    
    def _create_foot(self, parent: int, leg_name: str):
        """Create a spherical foot at the end of the tibia"""
        foot_visual = pb.createVisualShape(
            pb.GEOM_SPHERE,
            radius=0.025,
            visualFramePosition=[self.config.tibia_length, 0, 0],
            rgbaColor=[0.2, 0.2, 0.2, 1.0]  # Dark gray
        )
        
        foot = pb.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=foot_visual,
            basePosition=[self.config.tibia_length, 0, 0],
            baseInertialFrameOrientation=[0, 0, 0, 1],
            linkCollisionShapes=[],
            linkVisualShapes=[],
            linkMasses=[],
            linkPositions=[],
            linkOrientations=[],
            linkInertialFrameOrientation=[],
            parentObjectIndex=parent,
            parentLinkIndex=-1
        )
        
        return foot
    
    def set_joint_angles(self, angles: Dict[str, LegAngles]):
        """
        Set target joint angles for all legs.
        
        Args:
            angles: Dict mapping leg name to LegAngles from kinematics
        """
        if not self.robot_id:
            raise RuntimeError("Robot not created. Call create_robot() first.")
        
        for leg_name, leg_angles in angles.items():
            if leg_angles is None:
                continue
            
            # Set coxa angle (hip abduction/adduction)
            self._set_joint_angle(f"{leg_name}_coxa_joint", leg_angles.coxa)
            
            # Set femur angle (upper leg)
            self._set_joint_angle(f"{leg_name}_femur_joint", leg_angles.femur)
            
            # Set tibia angle (lower leg)
            self._set_joint_angle(f"{leg_name}_tibia_joint", leg_angles.tibia)
    
    def _set_joint_angle(self, joint_name: str, angle: float):
        """Set target position for a single joint"""
        if joint_name not in self.joints:
            return
        
        joint = self.joints[joint_name]
        joint.target_angle = angle
        
        pb.setJointMotorControl2(
            bodyIndex=self.robot_id,
            jointIndex=joint.joint_index,
            controlMode=pb.POSITION_CONTROL,
            targetPosition=angle,
            force=self.config.max_torque,
            positionGain=0.5,
            velocityGain=0.5
        )
    
    def get_joint_angles(self) -> Dict[str, LegAngles]:
        """
        Get current joint angles from simulation.
        
        Returns:
            Dict mapping leg name to LegAngles
        """
        if not self.robot_id:
            raise RuntimeError("Robot not created")
        
        result = {}
        for leg_name in ["FL", "FR", "BL", "BR"]:
            try:
                coxa = pb.getJointState(self.robot_id, self.joints[f"{leg_name}_coxa_joint"].joint_index)[0]
                femur = pb.getJointState(self.robot_id, self.joints[f"{leg_name}_femur_joint"].joint_index)[0]
                tibia = pb.getJointState(self.robot_id, self.joints[f"{leg_name}_tibia_joint"].joint_index)[0]
                
                result[leg_name] = LegAngles(coxa=coxa, femur=femur, tibia=tibia)
            except KeyError:
                result[leg_name] = None
        
        return result
    
    def get_robot_state(self) -> dict:
        """
        Get current state of the robot.
        
        Returns:
            Dict with position, orientation, velocity, etc.
        """
        if not self.robot_id:
            raise RuntimeError("Robot not created")
        
        pos, quat = pb.getBasePositionAndOrientation(self.robot_id)
        vel, ang_vel = pb.getBaseVelocity(self.robot_id)
        
        # Convert quaternion to Euler angles
        euler = pb.getEulerFromQuaternion(quat)
        
        return {
            "position": pos,
            "orientation_euler": euler,
            "orientation_quaternion": quat,
            "linear_velocity": vel,
            "angular_velocity": ang_vel,
            "joint_angles": self.get_joint_angles()
        }
    
    def check_stability(self) -> Tuple[bool, str]:
        """
        Check if the robot is stable (not fallen).
        
        Returns:
            Tuple of (is_stable, status_message)
        """
        state = self.get_robot_state()
        
        # Check if robot is upright
        roll, pitch, yaw = state["orientation_euler"]
        
        # Check if body height is reasonable
        body_height = state["position"][2]
        
        # Check if orientation is within bounds
        max_tilt = math.radians(45)  # 45 degrees
        
        is_stable = True
        issues = []
        
        if body_height < 0.1:
            is_stable = False
            issues.append(f"Body too low ({body_height:.2f}m)")
        
        if abs(roll) > max_tilt:
            is_stable = False
            issues.append(f"Excessive roll ({math.degrees(roll):.1f}°)")
        
        if abs(pitch) > max_tilt:
            is_stable = False
            issues.append(f"Excessive pitch ({math.degrees(pitch):.1f}°)")
        
        if is_stable:
            return True, "Robot is stable"
        else:
            return False, f"Unstable: {'; '.join(issues)}"
    
    def step(self, dt: float = 1.0/240.0):
        """
        Advance the simulation by one step.
        
        Args:
            dt: Time step (default 240Hz)
        """
        if not self.is_connected:
            raise RuntimeError("Not connected to PyBullet")
        
        pb.stepSimulation()
        self.simulation_time += dt
        
        # Process GUI events if in GUI mode
        if self.gui:
            # Allow user to quit with ESC
            keys = pb.getKeyboardEvents()
            if ord('q') in keys and keys[ord('q')] & pb.KEY_WAS_TRIGGERED:
                self.disconnect()
                raise KeyboardInterrupt("User pressed 'q' to quit")
    
    def step_physics(self, duration: float, angles: Dict[str, LegAngles] = None):
        """
        Run physics simulation for a duration while applying angles.
        
        Args:
            duration: How long to simulate (seconds)
            angles: Joint angles to apply (optional, uses current if None)
        """
        if angles:
            self.set_joint_angles(angles)
        
        steps = int(duration / (1.0/240.0))
        for _ in range(steps):
            self.step()
    
    def reset(self, position: Tuple[float, float, float] = (0, 0, 0.4),
              orientation: Tuple[float, float, float] = (0, 0, 0)):
        """
        Reset robot to initial position.
        
        Args:
            position: (x, y, z) starting position
            orientation: (roll, pitch, yaw) in radians
        """
        if not self.robot_id:
            raise RuntimeError("Robot not created")
        
        # Convert Euler to quaternion
        quat = pb.getQuaternionFromEuler(orientation)
        
        # Reset base position and velocity
        pb.resetBasePositionAndOrientation(self.robot_id, position, quat)
        pb.resetBaseVelocity(self.robot_id, [0, 0, 0], [0, 0, 0])
        
        # Reset all joints to neutral position
        neutral_angles = {
            "FL": LegAngles(coxa=0, femur=math.pi/4, tibia=-math.pi/2),
            "FR": LegAngles(coxa=0, femur=math.pi/4, tibia=-math.pi/2),
            "BL": LegAngles(coxa=0, femur=math.pi/4, tibia=-math.pi/2),
            "BR": LegAngles(coxa=0, femur=math.pi/4, tibia=-math.pi/2),
        }
        self.set_joint_angles(neutral_angles)
        
        # Let physics settle
        for _ in range(120):  # 0.5 seconds
            self.step()
        
        self.simulation_time = 0.0
        print(f"Robot reset to position {position}")
    
    def disconnect(self):
        """Disconnect from PyBullet"""
        if self.is_connected:
            pb.disconnect()
            self.is_connected = False
            print("PyBullet disconnected")
    
    @contextmanager
    def simulation_context(self):
        """Context manager for simulation lifecycle"""
        try:
            self.connect()
            self.create_robot()
            yield self
        finally:
            self.disconnect()
    
    def get_foot_positions(self) -> Dict[str, Tuple[float, float, float]]:
        """
        Get current world positions of all feet.
        
        Useful for debugging foot trajectory tracking.
        """
        if not self.robot_id:
            raise RuntimeError("Robot not created")
        
        foot_positions = {}
        
        for leg_name in ["FL", "FR", "BL", "BR"]:
            try:
                tibia_link_index = self._get_link_index(leg_name, "tibia")
                
                # Get world transform of tibia link
                link_state = pb.getLinkState(
                    self.robot_id, 
                    tibia_link_index,
                    computeForwardKinematics=True
                )
                
                world_pos = link_state[0]  # World position
                foot_positions[leg_name] = world_pos
            except (KeyError, IndexError):
                foot_positions[leg_name] = (0, 0, 0)
        
        return foot_positions
    
    def _get_link_index(self, leg_name: str, link_type: str) -> int:
        """Get the link index for a leg component"""
        # This is a workaround - link indices depend on creation order
        base_indices = {
            "FL": (0, 1, 2),
            "FR": (3, 4, 5),
            "BL": (6, 7, 8),
            "BR": (9, 10, 11),
        }
        
        link_indices = {"coxa": 0, "femur": 1, "tibia": 2}
        base_idx = base_indices[leg_name][link_indices[link_type]]
        
        # Add offset for right legs (they come after left legs in creation order)
        if leg_name in ["FR", "BR"]:
            base_idx += 12  # Offset for first two legs' visual links
        
        return base_idx


# Demo and testing functions
def demo_simple_movement():
    """Demonstrate simple robot movement in simulation"""
    print("=" * 60)
    print("PyBullet Simulation Demo")
    print("=" * 60)
    
    # Create simulation
    sim = PyBulletWrapper(gui=True)
    sim.connect()
    sim.create_robot()
    
    # Create kinematics
    kinematics = QuadrupedKinematics()
    
    # Neutral stance angles
    neutral_angles = {
        "FL": LegAngles(coxa=0, femur=0.5, tibia=-1.0),
        "FR": LegAngles(coxa=0, femur=0.5, tibia=-1.0),
        "BL": LegAngles(coxa=0, femur=0.5, tibia=-1.0),
        "BR": LegAngles(coxa=0, femur=0.5, tibia=-1.0),
    }
    
    # Set initial pose and let settle
    sim.set_joint_angles(neutral_angles)
    for _ in range(240):  # 1 second
        sim.step()
    
    print("\nStarting gait demonstration...")
    
    # Simple walking gait
    gait_period = 0.5
    steps_per_period = int(gait_period * 240)
    
    try:
        for cycle in range(4):  # 4 gait cycles
            print(f"\nGait cycle {cycle + 1}")
            
            # Swing front legs
            swing_angles = {
                "FL": LegAngles(coxa=0.3, femur=0.8, tibia=-1.2),
                "FR": LegAngles(coxa=-0.3, femur=0.8, tibia=-1.2),
                "BL": neutral_angles["BL"],
                "BR": neutral_angles["BR"],
            }
            sim.set_joint_angles(swing_angles)
            
            for _ in range(steps_per_period // 2):
                sim.step()
                if cycle == 0:
                    # Check stability on first cycle
                    stable, msg = sim.check_stability()
                    if not stable:
                        print(f"Warning: {msg}")
            
            # Swing back legs
            swing_angles = {
                "FL": neutral_angles["FL"],
                "FR": neutral_angles["FR"],
                "BL": LegAngles(coxa=0.3, femur=0.8, tibia=-1.2),
                "BR": LegAngles(coxa=-0.3, femur=0.8, tibia=-1.2),
            }
            sim.set_joint_angles(swing_angles)
            
            for _ in range(steps_per_period // 2):
                sim.step()
            
            # Return to neutral
            sim.set_joint_angles(neutral_angles)
            for _ in range(60):
                sim.step()
            
            # Check robot state
            state = sim.get_robot_state()
            stable, msg = sim.check_stability()
            print(f"  Robot height: {state['position'][2]:.3f}m, "
                  f"Orientation: R={math.degrees(state['orientation_euler'][0]):.1f}°, "
                  f"P={math.degrees(state['orientation_euler'][1]):.1f}°, "
                  f"Status: {msg}")
    
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    
    print("\nDemo complete!")
    sim.disconnect()


def demo_with_gait_controller():
    """Demonstrate using gait controller with PyBullet"""
    print("=" * 60)
    print("Gait Controller + PyBullet Demo")
    print("=" * 60)
    
    from .gait import GaitController, GaitConfig
    
    # Create simulation
    sim = PyBulletWrapper(gui=True)
    sim.connect()
    sim.create_robot()
    
    # Create gait controller
    kinematics = QuadrupedKinematics()
    config = GaitConfig(
        step_length=0.1,
        step_height=0.08,
        stance_ratio=0.6,
        cycle_period=0.4
    )
    gait = GaitController(kinematics, config)
    
    # Set neutral pose
    neutral_angles = {
        "FL": LegAngles(coxa=0, femur=0.4, tibia=-0.8),
        "FR": LegAngles(coxa=0, femur=0.4, tibia=-0.8),
        "BL": LegAngles(coxa=0, femur=0.4, tibia=-0.8),
        "BR": LegAngles(coxa=0, femur=0.4, tibia=-0.8),
    }
    sim.set_joint_angles(neutral_angles)
    
    # Let settle
    for _ in range(240):
        sim.step()
    
    print("\nWalking forward...")
    
    dt = 1.0 / 240
    velocity = (0.1, 0, 0)  # Move forward at 0.1 m/s
    
    try:
        for second in range(10):  # 10 seconds
            # Update gait
            target_angles = gait.update(dt, velocity)
            sim.set_joint_angles(target_angles)
            
            # Step simulation
            for _ in range(240):
                sim.step()
            
            # Check state
            state = sim.get_robot_state()
            stable, msg = sim.check_stability()
            print(f"  Second {second + 1}: Height={state['position'][2]:.3f}m, "
                  f"Stable={stable}")
            
            if not stable:
                print(f"  Warning: {msg}")
                break
    
    except KeyboardInterrupt:
        print("\nDemo interrupted")
    
    print("\nDemo complete!")
    sim.disconnect()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Dante PyBullet Simulation")
    parser.add_argument("--nogui", action="store_true", 
                       help="Run without GUI (headless)")
    parser.add_argument("--demo", type=str, default="simple",
                       choices=["simple", "gait"],
                       help="Which demo to run")
    
    args = parser.parse_args()
    
    if args.nogui:
        gui = False
    else:
        gui = True
    
    if args.demo == "simple":
        demo_simple_movement()
    else:
        demo_with_gait_controller()

