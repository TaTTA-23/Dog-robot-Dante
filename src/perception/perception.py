"""Perception module: handles sensors and basic object detection (stub)."""
from typing import List, Dict


class Perception:
    def __init__(self):
        # Sensor state placeholders
        self.ultrasonic: float = 0.0
        self.camera_feed: List[int] = []

    def scan_environment(self) -> Dict[str, float]:
        """Retorna leituras simples dos sensores (stub)."""
        # Em hardware real, aqui chamaria drivers/SDKs
        return {"ultrasonic": self.ultrasonic}

    def detect_objects(self) -> List[dict]:
        """Detecta objetos na imagem/câmera (stub).

        Retorna lista de dicionários com propriedades mínimas.
        """
        # placeholder: se houver um pixel alto, consideramos um objeto
        if not self.camera_feed:
            return []
        # encontrar índice do maior valor como 'objeto'
        max_idx = max(range(len(self.camera_feed)), key=lambda i: self.camera_feed[i])
        return [{"type": "unknown", "confidence": 0.5, "position": max_idx}]
