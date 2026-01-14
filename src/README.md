# Dante Robot — código fonte (src/)

Este diretório contém o esqueleto principal do software do robô "Dante" — uma
arquitetura modular em Python que separa percepção, locomoção, manipulação e o
"cérebro" de decisão.

Estrutura
```
src/
├── main.py                # Ponto de entrada do demo
├── brain/                 # Tomada de decisão (DanteBrain)
├── perception/            # Leitura e processamento de sensores
├── locomotion/            # Controle de movimento (walk/run/jump)
└── manipulation/          # Controle das garras (grab/release)
```

Como executar o demo localmente

Recomendo usar um virtualenv e executar com `PYTHONPATH=src` para que o
módulo `src` seja encontrado sem instalação:

```bash
python3 -m venv .venv
source .venv/bin/activate
# (opcional) instalar dependências do projeto se houver
PYTHONPATH=src python3 src/main.py
```

O demo executa duas missões de exemplo via `DanteBrain`:
- `GUIA_CEGO` — comportamento de guia (checa sensor ultrassônico e anda)
- `BUSCA_OBJETO` — busca objeto simples e tenta agarrar

Rodando os testes

Os testes do projeto são colocados em `tests/`. Para executar apenas os
testes do skeleton (sem depender de pacotes externos):

```bash
source .venv/bin/activate   # se estiver usando venv
pytest -q tests/test_system.py
```

Notas importantes

- O código aqui é um scaffold: métodos como `Locomotion.walk()` ou
  `Perception.detect_objects()` são stubs para facilitar testes locais. Substitua
  por implementação real ligada a drivers/hardware quando necessário.
- O nó ROS (`dante_node`) em `ros2_ws/` é um stub que também roda sem ROS2
  (imprime heartbeats). Para usar ROS2 propriamente, instale `rclpy` no ambiente
  ou estenda o `devcontainer` para incluir ROS2 (posso ajudar com isso).
- Para desenvolvimento em Codespaces/VS Code, veja o `.devcontainer/devcontainer.json`
  que instala dependências Python básicas e tenta instalar o pacote `python_ai`.

Contribuindo

- Adicione novas funcionalidades por módulo (ex.: novo node de visão em
  `perception/`) e escreva testes unitários em `tests/`.
- Considere adicionar CI (GitHub Actions) que rode `pytest` em cada PR.

Contato

Se quiser, eu posso:
- Estender o `devcontainer` para instalar ROS2 automaticamente.
- Adicionar GitHub Actions para testes automáticos.
- Criar exemplos de integração com câmera/simulador.
