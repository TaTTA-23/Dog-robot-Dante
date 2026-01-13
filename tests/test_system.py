import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from perception.perception import Perception
from locomotion.locomotion import Locomotion
from manipulation.manipulation import Manipulation
from brain.dante_brain import DanteBrain


def test_perception_detect_empty():
    p = Perception()
    assert p.detect_objects() == []


def test_locomotion_basic():
    l = Locomotion()
    assert l.walk() == "walking"
    assert l.run() == "running"


def test_brain_busca_objeto_no_object():
    p = Perception()
    l = Locomotion()
    m = Manipulation()
    brain = DanteBrain(p, l, m)
    assert brain.execute_mission("BUSCA_OBJETO") == "no_object_found"


def test_brain_guia_cego_obstacle():
    p = Perception()
    p.ultrasonic = 0.0  # obstacle too close
    l = Locomotion()
    m = Manipulation()
    brain = DanteBrain(p, l, m)
    assert brain.execute_mission("GUIA_CEGO") == "obstacle_too_close"
