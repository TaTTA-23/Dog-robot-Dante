# CHANGELOG

Todas as mudanças notáveis neste repositório serão documentadas neste arquivo.

## [Unreleased]

### Added

- Esqueleto ROS2: `ros2_ws/src/dante_bringup/` com `package.xml`, `setup.py`,
  `setup.cfg`, nó Python `dante_node.py` (stub que também roda sem ROS2) e
  launch file de exemplo.
- Pacote Python para prototipagem: `python_ai/` com utilitário de visão
  (`python_ai/vision.py`), testes pytest (`python_ai/tests/test_vision.py`),
  `requirements.txt` e `setup.py`.
- Devcontainer: `.devcontainer/devcontainer.json` para facilitar Codespaces/VS Code
  com instalação pós-criação de dependências Python.
- `.gitignore` adicionado e `.venv` removido do índice para evitar versionamento
  do ambiente virtual.

### Changed

- README principal atualizado com instruções básicas e resumo do scaffold.

### Testing

- Testes unitários Python (`python_ai`) executados localmente: todos passaram.

### Notes

- O nó ROS é um stub intencional — para desenvolvimento com ROS2 instale `rclpy`
  no ambiente ou estenda o devcontainer para incluir ROS2.
- Branch de trabalho: `feat/scaffold-ros2-pythonai-devcontainer`.

---

Sugestão de versão inicial: `0.1.0` após revisão e merge.
