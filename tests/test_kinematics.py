"""
Testes para o módulo de cinemática e locomoção do Dante.
"""

import math
import sys
sys.path.insert(0, '/home/codespace/Dog-robot-Dante/src')

from locomotion.kinematics import LegKinematics, LegAngles, QuadrupedKinematics
from locomotion.gait import GaitController, GaitConfig, GaitType, TrajectoryGenerator


def test_leg_kinematics_basic():
    """Teste básico da cinemática de uma pata"""
    print("=" * 60)
    print("TESTE: Cinemática Básica da Pata")
    print("=" * 60)
    
    leg = LegKinematics(l1=0.05, l2=0.10, l3=0.15, leg_name="TestLeg")
    target_x, target_y, target_z = 0.15, 0.0, -0.15
    
    print(f"Posição alvo: x={target_x}, y={target_y}, z={target_z}")
    angles = leg.calculate_angles(target_x, target_y, target_z)
    
    print(f"Ângulos calculados:")
    print(f"  Coxa (theta1): {math.degrees(angles.coxa):.2f}°")
    print(f"  Fêmur (theta2): {math.degrees(angles.femur):.2f}°")
    print(f"  Tíbia (theta3): {math.degrees(angles.tibia):.2f}°")
    
    assert -90 <= math.degrees(angles.coxa) <= 90, "Coxa angle out of range"
    assert -180 <= math.degrees(angles.femur) <= 0, "Femur angle out of range"
    assert -180 <= math.degrees(angles.tibia) <= 0, "Tibia angle out of range"
    
    print("✓ TESTE PASSOU: Cinemática básica funcionando\n")
    return True


def test_forward_kinematics():
    """Teste da cinemática direta"""
    print("=" * 60)
    print("TESTE: Cinemática Direta (Validação)")
    print("=" * 60)
    
    leg = LegKinematics(0.05, 0.10, 0.15, "TestFK")
    angles = LegAngles(coxa=math.radians(30), 
                       femur=math.radians(-45), 
                       tibia=math.radians(-60))
    pos = leg.calculate_position(angles)
    
    print(f"Ângulos de entrada: coxa=30°, femur=-45°, tibia=-60°")
    print(f"Posição calculada: x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}")
    
    assert pos.z < 0, "Foot should be below shoulder"
    assert abs(pos.x) > 0, "X position should be non-zero"
    
    print("✓ TESTE PASSOU: Cinemática direta validada\n")
    return True


def test_trajectory_generator():
    """Teste do gerador de trajetória elíptica"""
    print("=" * 60)
    print("TESTE: Trajetória Elíptica")
    print("=" * 60)
    
    config = GaitConfig(step_height=0.05, step_length=0.10)
    trajectory = TrajectoryGenerator(config)
    
    start = (0.0, 0.0, 0.0)
    end = (0.10, 0.0, 0.0)
    
    for phase in [0.0, 0.25, 0.5, 0.75, 1.0]:
        pos = trajectory.generate_elliptical_trajectory(start, end, phase)
        print(f"Phase {phase}: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")
        
        if phase == 0 or phase == 1:
            assert abs(pos[2]) < 0.001, f"Z should be 0 at phase {phase}"
        elif phase == 0.5:
            assert abs(pos[2] - 0.05) < 0.001, "Z should be max at phase 0.5"
    
    print("✓ TESTE PASSOU: Trajetória elíptica funcionando\n")
    return True


def test_quadruped_kinematics():
    """Teste do sistema de cinemática completo com 4 patas"""
    print("=" * 60)
    print("TESTE: Sistema de Cinemática Completo (4 patas)")
    print("=" * 60)
    
    quad = QuadrupedKinematics()
    
    positions = {
        "FL": (0.15, 0.08, -0.15),
        "FR": (0.15, -0.08, -0.15),
        "BL": (-0.15, 0.08, -0.15),
        "BR": (-0.15, -0.08, -0.15),
    }
    
    angles = quad.calculate_all_angles(positions)
    
    for leg_name, ang in angles.items():
        print(f"{leg_name}: coxa={math.degrees(ang.coxa):.1f}°, "
              f"femur={math.degrees(ang.femur):.1f}°, "
              f"tibia={math.degrees(ang.tibia):.1f}°")
    
    print("✓ TESTE PASSOU: Sistema de 4 patas funcionando\n")
    return True


def test_gait_controller():
    """Teste do controlador de gait"""
    print("=" * 60)
    print("TESTE: Controlador de Gait")
    print("=" * 60)
    
    quad = QuadrupedKinematics()
    config = GaitConfig(step_length=0.08, step_height=0.05)
    gait = GaitController(quad, config)
    
    for i in range(10):
        dt = 0.02
        velocity = (0.1, 0, 0)
        angles = gait.update(dt, velocity)
        if i == 0:
            print(f"Primeiro ciclo: {len(angles)} pernas controladas")
    
    gait.set_gait_type(GaitType.TROT)
    print("Gait alterado para TROT")
    
    print("✓ TESTE PASSOU: Controlador de gait funcionando\n")
    return True


def run_all_tests():
    """Executa todos os testes"""
    print("\n" + "=" * 60)
    print("EXECUTANDO TESTES DO MÓDULO DE CINEMÁTICA - DANTE")
    print("=" * 60 + "\n")
    
    tests = [
        test_leg_kinematics_basic,
        test_forward_kinematics,
        test_trajectory_generator,
        test_quadruped_kinematics,
        test_gait_controller,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            if test():
                passed += 1
        except Exception as e:
            print(f"✗ TESTE FALHOU: {test.__name__}")
            print(f"  Erro: {e}\n")
            failed += 1
    
    print("=" * 60)
    print(f"RESULTADO: {passed} passed, {failed} failed")
    print("=" * 60)
    
    return failed == 0


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
