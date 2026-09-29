"""Установка выбранного пакета без запуска робота и изменения служб."""
import argparse
import compileall
from datetime import datetime, timezone
import importlib
import json
import os
from pathlib import Path
import shutil
import sys

TEAM=Path('/home/ubuntu/ai-robot/team_wolf_setup')
PYTHON=Path('/home/ubuntu/ai-robot/venv/bin/python')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('release')
    args=p.parse_args()
    release=(TEAM/args.release).resolve(strict=True)
    if release.parent!=TEAM.resolve() or not release.name.startswith('training-'):
        raise ValueError('Пакет должен быть внутри team_wolf_setup/training-*')
    if not (release/'training_entry.py').is_file():
        raise ValueError('Нет точки входа пакета')
    sys.path.insert(0,str(TEAM/'perception_deps'))
    versions={}
    for name in ('numpy','scipy','cv2','aiortc','av','lz4.block'):
        module=importlib.import_module(name)
        versions[name]=getattr(module,'__version__','import_ok')
    if not compileall.compile_dir(release,quiet=1):
        raise ValueError('Ошибка компиляции установленного пакета')
    marker='# Managed Go2 autonomy launcher v1'
    launcher=TEAM/'go2-autonomy'
    if launcher.exists() and marker not in launcher.read_text():
        raise ValueError('Имя go2-autonomy уже занято другим файлом')
    current=TEAM/'autonomy-current'
    if current.exists() and not current.is_symlink():
        raise ValueError('autonomy-current уже занят обычным каталогом')
    previous=os.readlink(current) if current.is_symlink() else None
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    target=TEAM/('autonomy-current-'+stamp)
    target.symlink_to(release.name,target_is_directory=True)
    os.replace(target,current)
    content='''#!/bin/sh
# Managed Go2 autonomy launcher v1
set -eu
BASE=/home/ubuntu/ai-robot/team_wolf_setup
PY=/home/ubuntu/ai-robot/venv/bin/python
case "${1:-help}" in
  check) exec "$PY" -m json.tool "$BASE/pi-installation.json" ;;
  observe) exec "$PY" -u "$BASE/autonomy-current/training_entry.py" --observe ;;
  slalom|aframe|teeter|platforms) exec "$PY" -u "$BASE/autonomy-current/training_entry.py" --kind "$1" ;;
  *) echo 'Usage: go2-autonomy check | observe | slalom | aframe | teeter | platforms'; exit 2 ;;
esac
'''
    temp=TEAM/('go2-autonomy-'+stamp)
    temp.write_text(content,encoding='utf-8');temp.chmod(0o755);os.replace(temp,launcher)
    result={'installed_at_utc':stamp,'release':release.name,'previous_release':previous,
            'python':str(PYTHON),'dependencies':versions,'compile_ok':True,
            'motion_started':False,'autonomy_pass_verified':False,
            'profile_exists':(TEAM/'autonomy-profile.json').is_file(),
            'free_disk_bytes':shutil.disk_usage(TEAM).free,
            'note':'Установка пакета подтверждена; калибровка датчиков и автономный проход не подтверждены'}
    (TEAM/'pi-installation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
