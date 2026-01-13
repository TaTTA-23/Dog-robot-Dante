# Dog-robot-Dante

Este repositório contém esqueleto inicial para o projeto "Dog-robot Dante".

Conteúdo criado:
- `ros2_ws/` : workspace ROS2 com pacote mínimo `dante_bringup` (node stub e launch)
- `python_ai/` : pacote Python para prototipagem de IA com testes (pytest)
- `.devcontainer/` : configuração de devcontainer para Codespaces/VS Code

Para rodar os testes Python locally (assumindo Python 3.11):

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e python_ai
pip install -r python_ai/requirements.txt
pytest -q
```

O nó ROS (`dante_node`) é um stub que roda sem ROS instalado (imprime heartbeats),
mas suporta execução com ROS2 se `rclpy` estiver disponível.
# Dog-robot-Dante
 - Funcionalidades:
Proteger, guiar pessoas com deficiência visual, auxiliar na busca de objetos específicos como metais, animais perdidos ou pessoas.
- Locomoção: andar, correr, saltar, escalar.
- Percepção : sensores para detectar ambiente - câmeras, sensores de ultrassom e sensores de toque.
- Manipulação: pegar e manipular objetos com suas garras.
-  IA: tomar decisões e aprender com o ambiente.
