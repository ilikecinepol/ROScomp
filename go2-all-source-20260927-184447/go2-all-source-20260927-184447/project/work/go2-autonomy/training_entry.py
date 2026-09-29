"""Бортовой запуск отдельного испытания из панели; без команд при импорте."""
import argparse
import asyncio
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
import queue
import threading
from types import SimpleNamespace

TEAM=Path('/home/ubuntu/ai-robot/team_wolf_setup')
SDK=Path('/home/ubuntu/ai-robot')


def emit(value):
    print(json.dumps(value,ensure_ascii=False),flush=True)


class StatusOutput:
    """Медленный SSH не блокирует бортовой цикл движения: хранится последний статус."""
    def __init__(self):
        self.queue=queue.Queue(maxsize=1)
        self.thread=threading.Thread(target=self.work,daemon=True)
        self.thread.start()

    def put(self,value):
        try:self.queue.put_nowait(value)
        except queue.Full:
            try:self.queue.get_nowait()
            except queue.Empty:pass
            try:self.queue.put_nowait(value)
            except queue.Full:pass

    def work(self):
        while True:
            value=self.queue.get()
            if value is None:return
            try:emit(value)
            except (BrokenPipeError,OSError):return

    def close(self):
        self.put(None)
        self.thread.join(timeout=.2)


def service(action):
    subprocess.run(['sudo','-n','systemctl',action,'fleet-dog'],check=True,timeout=20,
                   stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)


def active():
    return subprocess.run(['systemctl','is-active','--quiet','fleet-dog'],timeout=5).returncode==0


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--kind',choices=['slalom','aframe','teeter','platforms'])
    p.add_argument('--stage-next',choices=['aframe'])
    p.add_argument('--check',action='store_true')
    p.add_argument('--observe',action='store_true')
    p.add_argument('--loop',action='store_true')
    p.add_argument('--console-control',action='store_true')
    args=p.parse_args()
    if args.loop and (args.check or args.observe or args.kind or args.stage_next):
        p.error('Полная петля запускается отдельно от других режимов')
    if not args.check and not args.observe and not args.kind and not args.loop:
        p.error('Нужен вид препятствия')
    from wolf_go2.profile import RobotProfile
    profile_path=TEAM/'autonomy-profile.json'
    profile=None
    if not args.observe:
        if not profile_path.is_file():
            emit({'type':'error','reason':'Автономное прохождение ещё не готово: отсутствует проверенный autonomy-profile.json. Требуются привязка координат, проверка свежести лидара, распознавание препятствий и испытание каждого навыка. «Снять датчики» работает без профиля и без движения. Пустой файл проблему не решит.'})
            return 2
        profile=RobotProfile.load(profile_path)
        errors=profile.readiness(profile.robot_id)
        if args.loop:
            missing=set(profile.policy.required_kinds)-set(profile.policy.validated_skills)
            if missing:errors.append('Для полной петли не проверены навыки: '+', '.join(sorted(missing)))
        # Эта панель предназначена именно для первичного испытания навыка.
        # Датчики и геометрия обязательны; прежний успешный проход не обязателен.
        # Успех испытания не дописывает validated_skills автоматически.
        if args.kind and args.kind not in profile.policy.validated_skills:
            emit({'type':'trial','reason':'Первичное испытание навыка '+args.kind+'. Датчики, геометрия и свободный путь остаются обязательными.'})
        if errors:
            emit({'type':'error','reason':'; '.join(errors)})
            return 2
        if args.check:
            emit({'type':'checked','reason':'Профиль прочитан. Свежие датчики будут проверены при запуске.','robot_id':profile.robot_id})
            return 0
    # На Pi уже установлен headless OpenCV для обработки без рабочего стола.
    dependencies=TEAM/'perception_deps'
    if dependencies.is_dir():
        sys.path.insert(0,str(dependencies))
    from wolf_go2.runtime import run_session
    import fcntl
    # Тот же замок, что у ручного управления. Другие сеансы не завершаем.
    with (TEAM/'teleop.lock').open('a') as lease:
        try:
            fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            emit({'type':'error','reason':'Завершите ручной сеанс: робот уже занят нашим клиентом.'})
            return 2
        restore=False
        try:
            if active():
                restore=True
                service('stop')
            stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
            output=TEAM/'autonomy_logs'/stamp
            statuses=StatusOutput()
            settings=SimpleNamespace(mode='observe' if args.observe else 'live',profile=None if args.observe else str(profile_path),robot_id=profile.robot_id if profile else 'pi-0008',
                sdk_dir=str(SDK),output=str(output),start_file=str(TEAM/('start-'+stamp)),
                duration=20. if args.observe else (profile.policy.max_mission_seconds if args.loop else 240.),trial_kind=args.kind,stage_next=args.stage_next,
                button_start=not args.observe,console_control=args.console_control or not args.observe,status_sink=statuses.put,
                commissioning=bool(args.kind) and not args.observe)
            emit({'type':'connecting','reason':'Соединяемся и проверяем свежие датчики','output':str(output)})
            try:report=asyncio.run(run_session(settings))
            finally:statuses.close()
            emit({'type':'result',**report})
            if args.observe: return 2 if report.get('fault') else 0
            return 2 if report.get('fault') or report.get('mission_phase')!='FINISHED' or report.get('stop_acknowledged') is not True else 0
        finally:
            if restore:
                try:
                    service('start')
                    emit({'type':'restored','ok':active()})
                except Exception as exc:
                    emit({'type':'error','reason':'Не удалось восстановить fleet-dog: '+str(exc)})


if __name__=='__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:
        emit({'type':'error','reason':type(exc).__name__+': '+str(exc)})
        raise SystemExit(2)
