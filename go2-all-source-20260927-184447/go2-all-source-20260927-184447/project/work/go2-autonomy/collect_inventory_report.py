"""Снимок программ и файлов Pi только для чтения, без подключения к Go2."""
import datetime
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys

root=Path('/home/ubuntu/ai-robot')
team=root/'team_wolf_setup'
def command(args):
    try:
        p=subprocess.run(args,capture_output=True,text=True,timeout=20)
        return {'exit':p.returncode,'stdout':p.stdout[:150000],'stderr':p.stderr[:3000]}
    except Exception as e:return {'error':str(e)}
def read(path):
    try:return Path(path).read_text(errors='replace')[:200000]
    except Exception as e:return {'error':str(e)}
def files(folder):
    if not folder.exists():return []
    return [{'name':p.name,'directory':p.is_dir(),'symlink':p.is_symlink(),
             'size':p.stat().st_size,'mtime_utc':datetime.datetime.fromtimestamp(p.stat().st_mtime,datetime.timezone.utc).isoformat()}
            for p in sorted(folder.iterdir()) if not p.name.startswith('.')]
def hashes(folder):
    return {str(p.relative_to(folder)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(folder.rglob('*.py')) if '__pycache__' not in p.parts}
def sessions(folder):
    result=[]
    if not folder.exists():return result
    for p in sorted(folder.iterdir()):
        if not p.is_dir():continue
        count=total=0
        for base,dirs,names in os.walk(p,followlinks=False):
            for name in names:
                f=Path(base)/name
                if f.is_symlink():continue
                try:total+=f.stat().st_size;count+=1
                except OSError:pass
        item={'name':p.name,'files':count,'bytes':total}
        for name in ['summary.json','run_status.json','report.json']:
            f=p/name
            if f.is_file() and f.stat().st_size<250000:
                try:item[name]=json.loads(f.read_text())
                except Exception as e:item[name]={'error':str(e)}
        result.append(item)
    return result

report={'collected_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'read_only':True,'robot_connection_opened':False,'python':sys.version,
        'hostname':command(['hostname']),'kernel':command(['uname','-a']),
        'os':read('/etc/os-release'),'board':read('/proc/device-tree/model'),
        'boot_id':read('/proc/sys/kernel/random/boot_id'),
        'cpu':command(['lscpu']),'memory':command(['free','-h']),
        'disk':command(['df','-h']),'uptime':command(['uptime']),
        'services':command(['systemctl','list-units','--type=service','--state=running','--no-pager','--plain']),
        'fleet':command(['systemctl','show','fleet-dog','-p','ActiveState','-p','SubState','-p','MainPID','-p','FragmentPath','-p','User','-p','WorkingDirectory']),
        'processes':command(['ps','-eo','pid,comm,etimes,%cpu,%mem']),
        'debian_packages':command(['dpkg-query','-W','-f=${binary:Package}\t${Version}\n']),
        'python_packages':sorted([{'name':d.metadata.get('Name',''),'version':d.version}
                         for d in importlib.metadata.distributions()],key=lambda d:d['name'].lower()),
        'perception_packages':sorted([{'name':d.metadata.get('Name',''),'version':d.version}
                         for d in importlib.metadata.distributions(path=[str(team/'perception_deps')])],key=lambda d:d['name'].lower()),
        'ros_opt':files(Path('/opt/ros')),'root_files':files(root),'team_files':files(team),
        'installed':read(team/'pi-installation.json'),
        'active_release':str((team/'autonomy-current').resolve()),
        'profile_exists':(team/'autonomy-profile.json').is_file(),
        'release_hashes':hashes(team/'autonomy-current'),
        'sdk_sha256':hashlib.sha256((root/'go2.py').read_bytes()).hexdigest(),
        'sessions':{name:sessions(team/name) for name in ['autonomy_logs','motion_logs','teleop_logs']}}
print(json.dumps(report,ensure_ascii=True))
