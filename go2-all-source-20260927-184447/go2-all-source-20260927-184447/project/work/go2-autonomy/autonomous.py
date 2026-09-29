"""Точка входа на Raspberry Pi. Без аргументов — справка, без движения."""
from pathlib import Path
import sys

# Дополнения устанавливались отдельно от штатного окружения организатора.
dependencies = Path.home() / 'ai-robot/team_wolf_setup/perception_deps'
if dependencies.is_dir():
    sys.path.insert(0, str(dependencies))

from wolf_go2.__main__ import main

if __name__ == '__main__':
    raise SystemExit(main())
