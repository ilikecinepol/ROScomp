"""Скачивание завершённых сеансов через SSH, с проверкой SHA256 и дозагрузкой."""
import base64
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import queue
import re
import subprocess
import tarfile
import tempfile
import threading

REMOTE = '/home/ubuntu/ai-robot/team_wolf_setup/teleop_logs'
SESSION = re.compile(r'^\d{8}T\d{6}(?:\.\d+)?Z$')
LIST_SESSIONS = r'''
import pathlib,json,re,os,stat
root=pathlib.Path('/home/ubuntu/ai-robot/team_wolf_setup/teleop_logs')
result=[]
for d in sorted(root.iterdir(),reverse=True):
 if not d.is_dir() or d.is_symlink() or not re.fullmatch(r'\d{8}T\d{6}(?:\.\d+)?Z',d.name): continue
 size=count=0
 approximate=False
 for directory,dirs,files in os.walk(d,followlinks=False):
  for name in files:
   try:
    info=os.stat(os.path.join(directory,name),follow_symlinks=False)
    if stat.S_ISREG(info.st_mode):
     size+=info.st_size
     count+=1
   except OSError: approximate=True
 complete=(d/'run_status.json').is_file()
 result.append(dict(name=d.name,complete=complete,size=size,count=count,approximate=approximate or not complete))
print(json.dumps(result))
'''
MANIFEST = '''
import pathlib,json,hashlib,sys
root=pathlib.Path('/home/ubuntu/ai-robot/team_wolf_setup/teleop_logs')
selected=set(json.load(sys.stdin))
result=[]
for d in sorted(root.iterdir(), reverse=True):
 if d.name not in selected: continue
 if not d.is_dir() or d.is_symlink(): continue
 if not (d/'run_status.json').is_file(): continue
 files=[]
 for p in sorted(d.rglob('*')):
  if not p.is_file() or p.is_symlink() or root.resolve() not in p.resolve().parents: continue
  h=hashlib.sha256()
  with p.open('rb') as f:
   for b in iter(lambda:f.read(1048576),b''): h.update(b)
  files.append(dict(path=p.relative_to(root).as_posix(),size=p.stat().st_size,sha256=h.hexdigest()))
 result.append(dict(name=d.name,files=files))
print(json.dumps(result))
'''
STREAM = '''
import sys,json,tarfile,pathlib
root=pathlib.Path('/home/ubuntu/ai-robot/team_wolf_setup/teleop_logs').resolve()
paths=json.load(sys.stdin)
with tarfile.open(fileobj=sys.stdout.buffer,mode='w|') as t:
 for name in paths:
  p=(root/name).resolve()
  if root not in p.parents or not p.is_file(): raise ValueError('Invalid path')
  t.add(p,arcname=name,recursive=False)
'''

# Этот код выполняется на Pi; список имён фиксируется при подтверждении в окне.
DELETE_HELPER = r'''
import pathlib,json,re,shutil,os

def delete_sessions(root, names):
 root=pathlib.Path(root)
 if not root.is_absolute() or root.is_symlink() or root.resolve()!=root:
  raise ValueError('Unsafe recording root')
 if not isinstance(names,list) or not names or len(names)!=len(set(names)):
  raise ValueError('Invalid session list')
 targets=[]
 for name in names:
  if not isinstance(name,str) or not re.fullmatch(r'\d{8}T\d{6}(?:\.\d+)?Z',name):
   raise ValueError('Invalid session name')
  d=root/name
  if d.is_symlink() or d.resolve().parent!=root:
   raise ValueError('Unsafe session path')
  if not d.exists(): continue
  marker=d/'run_status.json'
  if not d.is_dir() or marker.is_symlink() or not marker.is_file():
   raise ValueError('Session is not complete: '+name)
  status=json.loads(marker.read_text(encoding='utf-8'))
  if not isinstance(status,dict) or 'exit_code' not in status:
   raise ValueError('Invalid completion marker: '+name)
  for directory,dirs,files in os.walk(d,followlinks=False):
   for entry in dirs+files:
    child=pathlib.Path(directory)/entry
    if child.is_symlink() or getattr(child,'is_junction',lambda:False)():
     raise ValueError('Linked entry in session: '+name)
  targets.append(d)
 deleted=[]
 for d in targets:
  try:
   shutil.rmtree(d)
   deleted.append(d.name)
  except OSError as exc:
   return dict(deleted=deleted,error=str(exc))
 return dict(deleted=deleted,error=None)
'''
DELETE_SESSIONS = DELETE_HELPER + "\nimport sys\nprint(json.dumps(delete_sessions(pathlib.Path('" + REMOTE + "'), json.load(sys.stdin))))\n"


def ssh_command(host, key, code):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9.-]*', host):
        raise ValueError('Некорректный адрес Pi')
    payload = base64.b64encode(code.encode()).decode()
    return ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
            '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=3',
            '-o', 'IdentitiesOnly=yes', '-o', 'StrictHostKeyChecking=yes',
            '-i', str(key), 'ubuntu@'+host,
            'python3 -c "import base64;exec(base64.b64decode(\''+payload+'\'))"']

def destination(root, name):
    parts = PurePosixPath(name).parts
    if len(parts) < 2 or not SESSION.fullmatch(parts[0]) or any(
            p in ('..', '.') or '\\' in p or ':' in p for p in parts):
        raise ValueError('Недопустимое имя файла записи')
    result = root.joinpath(*parts)
    if root.resolve() not in result.resolve().parents:
        raise ValueError('Путь вне папки записей')
    return result

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for data in iter(lambda: f.read(1048576), b''):
            h.update(data)
    return h.hexdigest()

def matches(path, item):
    return path.is_file() and path.stat().st_size == item['size'] and digest(path) == item['sha256']

def transfer(host, key, root, items, report):
    missing = {i['path']: i for i in items if not matches(destination(root, i['path']), i)}
    report(f'Всего файлов: {len(items)}; осталось скачать: {len(missing)}')
    if not missing:
        return
    with tempfile.TemporaryFile() as errors:
        proc = subprocess.Popen(ssh_command(host, key, STREAM), stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=errors, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        try:
            proc.stdin.write(json.dumps(list(missing)).encode())
            proc.stdin.close()
            completed = 0
            with tarfile.open(fileobj=proc.stdout, mode='r|') as archive:
                for member in archive:
                    item = missing.get(member.name)
                    if not item or not member.isfile() or member.size != item['size']:
                        raise ValueError('Состав записи изменился: обновите список')
                    target = destination(root, member.name)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    part = target.with_name(target.name+'.part')
                    with archive.extractfile(member) as source, part.open('wb') as output:
                        while True:
                            block = source.read(1024*1024)
                            if not block: break
                            output.write(block)
                    if not matches(part, item):
                        raise ValueError('Неполный файл: '+member.name)
                    os.replace(part, target)
                    completed += 1
                    report(f'Скачано {completed}/{len(missing)}: {member.name}')
            if proc.wait(timeout=20) != 0 or completed != len(missing):
                raise RuntimeError('Передача прервана')
        except Exception as exc:
            proc.kill()
            proc.wait()
            errors.seek(0)
            detail = errors.read().decode(errors='replace')[-1000:]
            raise RuntimeError(f'{exc}. {detail}\nПовторите скачивание: проверенные файлы сохраняются.') from exc
        finally:
            proc.stdout.close()

def open_downloads(parent, host, key, folder, config_path=None):
    import tkinter as tk
    from tkinter import ttk
    win = tk.Tk() if parent is None else tk.Toplevel(parent)
    win.title('Go2 — скачать записи на ПК')
    win.geometry('940x600')
    win.minsize(700, 420)
    events = queue.Queue()
    sessions = []
    busy = False
    address_row = ttk.Frame(win, padding=10)
    address_row.pack(fill='x')
    ttk.Label(address_row, text='IP Raspberry Pi:').pack(side='left')
    address = tk.StringVar(value=host or '')
    address_entry = ttk.Entry(address_row, textvariable=address, width=22)
    address_entry.pack(side='left', padx=8)
    def paste_ip():
        try: address.set(win.clipboard_get().strip())
        except tk.TclError: status.set('В буфере обмена нет текста.')
    paste = ttk.Button(address_row, text='Вставить IP', command=paste_ip)
    paste.pack(side='left')
    ttk.Label(address_row, text='VPN должен быть подключён').pack(side='right')
    ttk.Label(win, text='Все сеансы • свежие сверху • время в UTC', padding=10).pack()
    listing = tk.Listbox(win, selectmode='extended', exportselection=False)
    listing.pack(fill='both', expand=True, padx=10)
    status = tk.StringVar(value='Нажмите «Обновить список». Текущий сеанс появится после завершения.')
    ttk.Label(win, textvariable=status, wraplength=750, padding=10).pack(fill='x')
    ttk.Label(win, text=str(folder), wraplength=750).pack()
    row = ttk.Frame(win, padding=10)
    row.pack(fill='x')
    def start(job):
        nonlocal busy
        if busy: return
        busy = True
        for b in buttons: b.configure(state='disabled')
        address_entry.configure(state='disabled')
        paste.configure(state='disabled')
        def worker():
            try: job()
            except Exception as exc: events.put(('error', str(exc)))
            finally: events.put(('done', None))
        threading.Thread(target=worker, daemon=True).start()
    def refresh():
        nonlocal host
        if busy: return
        import ipaddress
        try:
            host = str(ipaddress.IPv4Address(address.get().strip()))
        except ValueError:
            status.set('Введите актуальный IPv4 Raspberry Pi и нажмите «Обновить список».')
            return
        if config_path:
            try:
                config_path.write_text(json.dumps({'Ip': host}, ensure_ascii=False, indent=2), encoding='utf-8')
            except OSError as exc:
                status.set('Не удалось сохранить IP: ' + str(exc)); return
        status.set('Получаем список и размеры файлов без вычисления контрольных сумм…')
        def job():
            r = subprocess.run(ssh_command(host, key, LIST_SESSIONS), capture_output=True,
                timeout=25, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            if r.returncode: raise RuntimeError(r.stderr.decode(errors='replace')[-1000:])
            data = json.loads(r.stdout)
            for s in data:
                if not SESSION.fullmatch(s['name']): raise ValueError('Неверное имя сеанса')
            events.put(('list', data))
        start(job)
    def download(all_sessions=False):
        indexes = range(len(sessions)) if all_sessions else listing.curselection()
        chosen = [sessions[i] for i in indexes if sessions[i]['complete']]
        if not chosen:
            status.set('Выберите завершённый сеанс. Незавершённые пока не скачиваются.'); return
        def job():
            events.put(('status', 'Подготавливаем контрольные суммы только выбранных сеансов…'))
            r = subprocess.run(ssh_command(host, key, MANIFEST),
                input=json.dumps([s['name'] for s in chosen]).encode(), capture_output=True,
                timeout=300, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            if r.returncode: raise RuntimeError(r.stderr.decode(errors='replace')[-1000:])
            prepared = json.loads(r.stdout)
            if {s['name'] for s in prepared} != {s['name'] for s in chosen}:
                raise RuntimeError('Список изменился: обновите сеансы')
            items = [f for s in prepared for f in s['files']]
            transfer(host, key, folder, items, lambda s: events.put(('status', s)))
            events.put(('status', f'Готово: {len(chosen)} сеансов, {len(items)} файлов. SHA256 проверены.'))
        start(job)
    def delete_all():
        if busy: return
        from tkinter import messagebox
        chosen = [dict(s) for s in sessions if s['complete']]
        if not chosen:
            status.set('Нет завершённых сеансов для удаления.'); return
        size = sum(s['size'] for s in chosen) / 1024**2
        text = (f'Удалить с Pi {host} все завершённые записи из текущего списка?\n\n'
                f'Сеансов: {len(chosen)}; объём: {size:.2f} МиБ.\n'
                'Будут удалены в том числе НЕ СКАЧАННЫЕ записи. Восстановить их этой программой нельзя.\n'
                'Файлы на ПК, текущая запись и программы на Pi останутся.\n\n'
                'Папка: ' + REMOTE)
        if not messagebox.askyesno('Удалить записи с Pi', text, parent=win, default='no', icon='warning'):
            return
        def job():
            events.put(('status', 'Удаляем завершённые записи с Pi…'))
            r = subprocess.run(ssh_command(host, key, DELETE_SESSIONS),
                input=json.dumps([s['name'] for s in chosen]).encode(), capture_output=True,
                timeout=300, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            if r.returncode:
                raise RuntimeError('Удаление не подтверждено; обновите список. ' + r.stderr.decode(errors='replace')[-1000:])
            events.put(('deleted', json.loads(r.stdout)))
        start(job)

    buttons = []
    for label, action in [('Обновить список', refresh), ('Скачать выбранное', download),
                          ('Скачать всё', lambda: download(True))]:
        b = ttk.Button(row, text=label, command=action); b.pack(side='left', padx=4); buttons.append(b)
    delete_row = ttk.Frame(win, padding=(10, 0, 10, 10))
    delete_row.pack(fill='x')
    delete_button = ttk.Button(delete_row, text='Удалить все завершённые записи с Pi…', command=delete_all)
    delete_button.pack(side='left', padx=4)
    buttons.append(delete_button)
    ttk.Label(delete_row, text='Включая не скачанные. Только после подтверждения.').pack(side='left', padx=8)
    def show_folder():
        folder.mkdir(parents=True, exist_ok=True)
        os.startfile(folder)
    ttk.Button(row, text='Открыть папку', command=show_folder).pack(side='left', padx=4)
    def poll():
        nonlocal sessions, busy
        while not events.empty():
            kind, data = events.get_nowait()
            if kind == 'list':
                sessions = data
                listing.delete(0, 'end')
                for s in sessions:
                    label = 'завершён' if s['complete'] else 'не завершён / прерван'
                    prefix = '≈' if s.get('approximate') else ''
                    listing.insert('end', f"{s['name']} — {prefix}{s['size']/1024**2:.2f} МиБ — {s['count']} файлов — {label}")
                completed = [i for i,s in enumerate(sessions) if s['complete']]
                if completed: listing.selection_set(completed[0])
                status.set(f'Найдено: {len(sessions)}. Для скачивания доступны завершённые сеансы.')
            elif kind == 'deleted':
                deleted = set(data.get('deleted', []))
                for i in reversed(range(len(sessions))):
                    if sessions[i]['name'] in deleted:
                        listing.delete(i)
                        sessions.pop(i)
                status.set(f'Удалено сеансов: {len(deleted)}. ' +
                    ('Ошибка: ' + str(data['error']) + '. Обновите список.' if data.get('error') else 'Файлы на ПК сохранены.'))
            elif kind == 'done':
                busy = False
                for b in buttons: b.configure(state='normal')
                address_entry.configure(state='normal')
                paste.configure(state='normal')
            else: status.set(data)
        win.after(100, poll)
    win.protocol('WM_DELETE_WINDOW', lambda: status.set('Дождитесь завершения передачи.') if busy else win.destroy())
    def changed(*_):
        nonlocal sessions
        if busy: return
        sessions = []
        listing.delete(0, 'end')
        status.set('IP изменён. Нажмите «Обновить список».')
    address.trace_add('write', changed)
    poll()
    refresh()
    return win


def saved_host(config_path):
    """Читаем тот же последний адрес, что использует панель подключения."""
    try:
        data = json.loads(config_path.read_text(encoding='utf-8-sig'))
        return str(data.get('Ip') or '').strip()
    except (OSError, ValueError, AttributeError):
        return ''


def main():
    import argparse
    workspace = Path(__file__).resolve().parents[2]
    base = workspace / 'outputs/go2-teleop'
    parser = argparse.ArgumentParser(description='Отдельное окно скачивания Go2, без управления роботом')
    parser.add_argument('--host')
    args = parser.parse_args()
    config = base / 'go2-quick-config.json'
    host = args.host or saved_host(config) or os.environ.get('GO2_PI_IP', '')
    win = open_downloads(None, host, Path.home()/'.ssh/go2-pi-0008/id_ed25519',
                         base/'recordings/teleop_logs', config_path=config)
    win.mainloop()


if __name__ == '__main__':
    try:
        main()
    except Exception:
        import traceback
        from tkinter import messagebox
        detail = traceback.format_exc()
        log = Path(__file__).resolve().with_name('download-startup-error.log')
        log.write_text(detail, encoding='utf-8')
        messagebox.showerror('Go2 — ошибка запуска', 'Не удалось открыть окно скачивания.\n' + detail[-1600:])
