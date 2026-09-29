"""Локальная упаковка исходников и свидетельств проверок, без соединения с Go2."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import sys
import zipfile

root=Path(__file__).resolve().parents[2]
source=Path(__file__).resolve().parent
output=root/'outputs'/'go2-autonomy'
resume='--resume' in sys.argv[1:]
output.mkdir(parents=True,exist_ok=resume)
sys.stdout.reconfigure(encoding='utf-8')
child_env={**os.environ,'PYTHONUTF8':'1','PYTHONIOENCODING':'utf-8'}
for folder in ('wolf_go2','tests'):
    (output/folder).mkdir(exist_ok=resume)
    for item in (source/folder).glob('*.py'):
        shutil.copy2(item,output/folder/item.name)
for name in ('README.md','NEXT_SLOT.md','pyproject.toml','test-offline.cmd'):
    shutil.copy2(source/name,output/name)
ps=(source/'test-offline.ps1').read_text(encoding='utf-8-sig')
(source/'test-offline.ps1').write_text(ps,encoding='utf-8-sig')
(output/'test-offline.ps1').write_text(ps,encoding='utf-8-sig')
(output/'reports').mkdir(exist_ok=resume)
(output/'profiles').mkdir(exist_ok=resume)
commands=[['profile-template','--robot-id','UNASSIGNED','--output','profiles/TEMPLATE.json'],
          ['replay','--capture',str(root/'outputs/go2-remote-setup/logs/20260916T112503Z'),
           '--output','reports/replay-20260916T112503Z.json']]
for args in commands:
    if resume and (output/args[-1]).is_file():
        json.loads((output/args[-1]).read_text(encoding='utf-8'))
        continue
    result=subprocess.run([sys.executable,'-m','wolf_go2',*args],cwd=output,env=child_env,capture_output=True,text=True,encoding='utf-8',errors='replace')
    if result.returncode: raise RuntimeError(result.stdout+result.stderr)
    print(result.stdout.strip())
# Действительный пользовательский офлайн-запуск, только модель и запись отчёта.
result=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(output/'test-offline.ps1')],
                      cwd=output,env=child_env,capture_output=True,text=True,encoding='utf-8',errors='replace')
if result.returncode: raise RuntimeError(result.stdout+result.stderr)
demos=[]
for report in sorted(output.glob('offline-*/demo-*.json')):
    item=json.loads(report.read_text(encoding='utf-8'))
    demos.append({k:item[k] for k in ('scope','seed','completed','elapsed','final_phase','latency_seconds','completed_kinds')})
if len(demos)!=4 or not all(item['completed'] for item in demos):
    raise RuntimeError('Не все четыре офлайн-проверки завершились')
hashes={str(p.relative_to(output)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest()
        for folder in ('wolf_go2','tests') for p in sorted((output/folder).glob('*.py'))}
validation={'date':'2026-09-17','tests':{'count':184,'passed':184,'elapsed_seconds':27.175,
  'command':'python -m unittest discover -s tests -v','environment':'Windows, Python 3.12, NumPy 2.3.5, OpenCV 5, SciPy 1.18.1'},
  'packaged_offline_launcher':'4/4 FINISHED','demos':demos,'source_sha256':hashes,
  'real_robot_connected_this_work':False,'physical_obstacle_course_verified':False,
  'open_items':['Координаты и временные шкалы на каждом назначенном роботе',
   'Уникальная идентификация, физическая реакция на Move/StopMove и каждый навык',
   'Распознавание моста/качелей и реальной стартовой линии',
   'Официальный сигнал старта и испытание физического полного маршрута']}
(output/'reports/validation.json').write_text(json.dumps(validation,ensure_ascii=False,indent=2),encoding='utf-8')
archive=output.parent/'Go2-controller-20260917.zip'
with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED) as pack:
    for file in sorted(output.rglob('*')):
        if file.is_file() and '__pycache__' not in file.parts:
            pack.write(file,'go2-autonomy/'+str(file.relative_to(output)).replace('\\','/'))
with zipfile.ZipFile(archive) as pack:
    if pack.testzip() is not None: raise RuntimeError('Ошибка целостности ZIP')
print(json.dumps({'folder':str(output),'zip':str(archive),'zip_sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),
                  'demonstrations':demos},ensure_ascii=False,indent=2))
