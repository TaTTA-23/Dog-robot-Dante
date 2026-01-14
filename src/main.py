"""Ponto de entrada simples para o Dante Robot skeleton."""
from brain.dante_brain import DanteBrain
from perception.perception import Perception
from locomotion.locomotion import Locomotion
from manipulation.manipulation import Manipulation


def run_demo():
    p = Perception()
    l = Locomotion()
    m = Manipulation()
    brain = DanteBrain(p, l, m)

    print("Running demo missions...")
    print(brain.execute_mission("GUIA_CEGO"))
    print(brain.execute_mission("BUSCA_OBJETO"))


if __name__ == '__main__':
    run_demo()
