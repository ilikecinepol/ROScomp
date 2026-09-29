"""Панель отдельных бортовых испытаний. Сеть выполняется вне потока Tk."""
import argparse
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import queue
import subprocess
import tempfile
import threading
import zipfile

ROOT=Path(__file__).resolve().parent
WORKSPACE=ROOT.parent.parent
TEAM='/home/ubuntu/ai-robot/team_wolf_setup'
LABELS={'slalom':'Змейка','aframe':'Горка','teeter':'Качели','platforms':'Рампа'}


def source_files():
    return [ROOT/'autonomous.py',ROOT/'training_entry.py',*sorted((ROOT/'wolf_go2').glob('*.py'))]


def make_package(destination):
    with zipfile.ZipFile(destination,'w',zipfile.ZIP_DEFLATED) as archive:
        for path in source_files():
            archive.write(path,path.relative_to(ROOT).as_posix())
    return hashlib.sha256(Path(destination).read_bytes()).hexdigest()[:20]


class Panel:
    def __init__(self,root,host=''):
        import tkinter as tk
        from tkinter import ttk
        self.root=root
        root.title('Go2 — отдельные автономные испытания')
        root.geometry('860x580')
        self.bus=queue.Queue()
        self.process=None
        self.busy=False
        self.cancel=threading.Event()
        self.closing=False
        self.config=WORKSPACE/'outputs/go2-teleop/go2-quick-config.json'
        if not host:
            try: host=json.loads(self.config.read_text(encoding='utf-8-sig')).get('Ip','')
            except (OSError,ValueError): pass
        self.ip=tk.StringVar(value=host)
        self.next=tk.BooleanVar(value=True)
        self.status=tk.StringVar(value='Выберите препятствие. Прохождение на физическом роботе ещё требует испытаний.')
        frame=ttk.Frame(root,padding=16);frame.pack(fill='both',expand=True)
        row=ttk.Frame(frame);row.pack(fill='x')
        ttk.Label(row,text='IP Raspberry Pi:').pack(side='left')
        self.entry=ttk.Entry(row,textvariable=self.ip,width=20);self.entry.pack(side='left',padx=8)
        ttk.Button(row,text='Вставить IP',command=self.paste).pack(side='left')
        self.check=ttk.Button(row,text='Подключить / проверить',command=lambda:self.start(None));self.check.pack(side='right')
        self.observe=ttk.Button(row,text='Автоподготовка / датчики',command=lambda:self.start('observe'));self.observe.pack(side='right',padx=5)
        ttk.Label(frame,text='Автономные навыки ещё проходят разработку. «Снять датчики» — проверка без движения. Кнопки препятствий запускают испытание только после проверки бортового профиля.',wraplength=810).pack(anchor='w',pady=14)
        buttons=ttk.Frame(frame);buttons.pack(fill='x')
        self.buttons=[]
        for kind,label in LABELS.items():
            button=ttk.Button(buttons,text=label,command=lambda k=kind:self.start(k))
            button.pack(side='left',expand=True,fill='x',ipady=12,padx=3);self.buttons.append(button)
        self.checkbox=ttk.Checkbutton(frame,text='После змейки подойти и выровняться перед горкой',variable=self.next)
        self.checkbox.pack(anchor='w',pady=10)
        self.loop=ttk.Button(frame,text='Вся петля — тренировочное испытание',command=lambda:self.start('loop'))
        self.loop.pack(fill='x',pady=(0,8))
        tk.Button(frame,text='СТОП',bg='#b62924',fg='white',font=('Segoe UI',17,'bold'),command=self.stop).pack(fill='x',ipady=8)
        ttk.Label(frame,textvariable=self.status,wraplength=810).pack(anchor='w',pady=12)
        self.log=tk.Text(frame,height=13,state='disabled',wrap='word');self.log.pack(fill='both',expand=True)
        ttk.Label(frame,text='Завершите ручное управление перед запуском. Закрытие панели останавливает испытание.',wraplength=810).pack(anchor='w',pady=8)
        root.protocol('WM_DELETE_WINDOW',self.close)
        root.bind('<Escape>',lambda _:self.stop())
        root.after(100,self.poll)

    def paste(self):
        if not self.busy:
            try:self.ip.set(self.root.clipboard_get().strip())
            except Exception:pass

    def start(self,kind):
        if self.busy:return
        try:host=str(ipaddress.IPv4Address(self.ip.get().strip()))
        except ValueError:
            self.status.set('Введите корректный IPv4');return
        self.config.write_text(json.dumps({'Ip':host}),encoding='utf-8')
        self.busy=True;self.cancel.clear()
        for b in [*self.buttons,self.loop,self.check,self.observe,self.entry,self.checkbox]:b.configure(state='disabled')
        self.status.set('Загружаем текущую версию…')
        stage='aframe' if kind=='slalom' and self.next.get() else None
        threading.Thread(target=self.worker,args=(host,kind,stage),daemon=True).start()

    def worker(self,host,kind,stage):
        key=os.environ.get('GO2_PI_KEY',str(Path.home()/'.ssh/go2-pi-0008/id_ed25519'))
        options=['-o','BatchMode=yes','-o','ConnectTimeout=8','-o','IdentitiesOnly=yes','-o','StrictHostKeyChecking=yes',
                 '-o','ServerAliveInterval=2','-o','ServerAliveCountMax=3','-i',key]
        target='ubuntu@'+host
        flags=getattr(subprocess,'CREATE_NO_WINDOW',0)
        try:
            if not Path(key).is_file():raise RuntimeError('Не найден SSH-ключ: '+key)
            with tempfile.TemporaryDirectory(prefix='go2-training-') as directory:
                archive=Path(directory)/'training.zip'
                digest=make_package(archive)
                remote=TEAM+'/training-'+digest
                commands=[['scp',*options,str(archive),target+':'+remote+'.zip'],
                    ['ssh',*options,target,'mkdir -p '+remote+' && python3 -m zipfile -e '+remote+'.zip '+remote]]
                for command in commands:
                    if self.cancel.is_set():return
                    result=subprocess.run(command,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=35,creationflags=flags)
                    if result.returncode:raise RuntimeError(result.stderr.strip() or 'Не удалось загрузить программу')
                if self.cancel.is_set():return
                command=TEAM.replace('/team_wolf_setup','')+'/venv/bin/python -u '+remote+'/training_entry.py'
                command+= ' --check' if kind is None else (' --observe --console-control' if kind=='observe' else (' --loop' if kind=='loop' else ' --kind '+kind))
                if stage:command+=' --stage-next '+stage
                process=subprocess.Popen(['ssh',*options,target,command],stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace',bufsize=1,creationflags=flags)
                self.process=process
                if self.cancel.is_set():self.send_stop(process)
                for line in process.stdout:
                    self.bus.put(('line',line.rstrip()))
                code=process.wait()
                self.bus.put(('exit',code))
        except Exception as exc:self.bus.put(('error',str(exc)))
        finally:
            self.process=None
            self.bus.put(('done',None))

    @staticmethod
    def send_stop(process):
        try:
            process.stdin.write('STOP\n');process.stdin.flush()
        except (OSError,ValueError):pass

    def stop(self):
        self.cancel.set()
        process=self.process
        if process:threading.Thread(target=self.send_stop,args=(process,),daemon=True).start()
        self.status.set('Остановка запрошена; ждём подтверждения с Pi.' if self.busy else 'Испытание не запущено.')

    def close(self):
        if not self.busy:self.root.destroy();return
        self.closing=True;self.stop()

    def poll(self):
        try:
            while True:
                kind,value=self.bus.get_nowait()
                if kind=='line':
                    try:
                        item=json.loads(value)
                        value=item.get('reason') or json.dumps(item,ensure_ascii=False)
                        if item.get('type')=='status':
                            value=item['phase']+': '+('; '.join(item.get('health',[])) or value)
                        if item.get('type')=='result':
                            value=('Датчики записаны: '+str(item.get('sensor_report',{})) if item.get('scope')=='observation_only' else ('Контроллер завершил испытание. Пройдено: '+', '.join(LABELS.get(k,k) for k in item.get('completed',[])) if item.get('mission_phase')=='FINISHED' and not item.get('fault') else 'Испытание не завершено: '+str(item.get('fault') or item.get('mission_phase'))))
                            if item.get('scope')=='observation_only':
                                audit=item.get('packet_audit',{})
                                scene=item.get('scene_audit',{})
                                value=('Диагностика завершена. Кандидатов стоек: '+str(len(scene.get('pole_candidates',[])))+'. '
                                    +'Пакетов лидара: '+str(audit.get('packets',0))+', разных меток времени: '+str(audit.get('distinct_stamps',0))+'. '
                                    +'Маршрут не подтверждён. '+str(scene.get('reason','')))
                                states={'no_recent_packets':'Пакеты лидара перестали приходить.',
                                    'content_progress_not_observed':'Изменение содержимого лидара не подтверждено.',
                                    'content_changes_source_time_unavailable':'Данные лидара меняются, но точное время съёмки недоступно.',
                                    'content_changes_time_not_calibrated':'Данные лидара меняются; часы источника ещё не сопоставлены с Pi.'}
                                value=states.get(audit.get('transport_status'),'')+' '+value
                                prep=item.get('preparation')
                                if prep:
                                    value=('Автоподготовка завершена без движения. Стойки камеры: '+str(prep['camera_poles'])
                                           +', группы лидара: '+str(prep['lidar_row_hypotheses'])+'.\n'
                                           +'Для старта осталось:\n• '+'\n• '.join(prep['blockers']))
                    except (ValueError,TypeError,AttributeError):pass
                    self.status.set(value)
                elif kind=='error':self.status.set('Ошибка: '+value)
                elif kind=='exit':
                    if value:self.status.set('Запуск не завершён. Причина — в журнале ниже.')
                    continue
                elif kind=='done':
                    self.busy=False
                    for b in [*self.buttons,self.loop,self.check,self.observe,self.entry,self.checkbox]:b.configure(state='normal')
                    if self.closing:self.root.destroy();return
                    continue
                self.log.configure(state='normal');self.log.insert('end',str(value)+'\n');self.log.see('end');self.log.configure(state='disabled')
        except queue.Empty:pass
        self.root.after(100,self.poll)


def main():
    import tkinter as tk
    parser=argparse.ArgumentParser();parser.add_argument('--host',default='')
    args=parser.parse_args()
    root=tk.Tk();Panel(root,args.host);root.mainloop()


if __name__=='__main__':main()
