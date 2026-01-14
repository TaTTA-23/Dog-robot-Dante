"""DanteBrain - camada de decisão simples que coordena módulos."""
from typing import Any


class DanteBrain:
    def __init__(self, perception: Any, locomotion: Any, manipulation: Any):
        self.perception = perception
        self.locomotion = locomotion
        self.manipulation = manipulation

    def execute_mission(self, mission_type: str) -> str:
        """Executa missão simples com base no tipo fornecido.

        Missões suportadas (exemplos):
         - GUIA_CEGO: guia com cuidado (usa walk)
         - BUSCA_OBJETO: busca objetos e tenta pegar
        """
        if mission_type == "GUIA_CEGO":
            # Exemplo simples: checar obstáculos via perception e andar
            sensors = self.perception.scan_environment()
            if sensors.get("ultrasonic", 0) < 0.1:
                return "obstacle_too_close"
            return self.locomotion.walk()

        if mission_type == "BUSCA_OBJETO":
            objs = self.perception.detect_objects()
            if not objs:
                return "no_object_found"
            grabbed = self.manipulation.grab()
            return "object_acquired" if grabbed else "grab_failed"

        return "unknown_mission"
