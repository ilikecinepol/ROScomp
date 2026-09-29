"""Бортовой адаптер WebRTC. Импорт и офлайн-команды не открывают соединение.

Никаких IP, ключей или файлов арены: соединение через выданный go2.py.
Режим observe не отправляет Sport-команд. Live запускается только явно.
"""
import asyncio
from dataclasses import asdict
import importlib
import json
import math
from pathlib import Path
import secrets
import signal
import sys
import time

import numpy as np
from .consistency import ConsistencyMonitor
from .control import CommandGate, CommandPump, acknowledged
from .journal import Journal, jsonable
from .mission import MissionController
from .models import Decision, Policy
from .perception import PerceptionPipeline
from .profile import RobotProfile
from .sensors import SensorMonitor, Stream

TOPICS=('LF_SPORT_MOD_STATE','LOW_STATE','ROBOTODOM','ULIDAR_STATE','ULIDAR_ARRAY')

def native_points(message):
    """Ожидаем проверенный формат native, mesh/байты не угадываем."""
    points=message['data']['data']['points']
    if not isinstance(points,np.ndarray) or points.ndim!=2 or points.shape[1]!=3:
        raise ValueError('Нет массива native Nx3')
    if not 1<=len(points)<=500000 or not np.isfinite(points).all():
        raise ValueError('Размер или значения облака вне контракта')
    return points.copy()

class Runtime:
    def __init__(self, profile, robot_id, journal, boot_id, live=False, clock=time.monotonic):
        self.profile,self.robot_id,self.journal=profile,robot_id,journal
        self.policy=profile.policy if profile else Policy()
        self.boot_id,self.live,self.clock=boot_id,live,clock
        self.monitor=SensorMonitor()
        self.pipeline=PerceptionPipeline()
        self.consistency=ConsistencyMonitor()
        self.mission=MissionController(self.policy)
        self.gate=CommandGate(self.policy)
        self.ending=asyncio.Event()
        self.points=self.image=None
        self.image_received=-math.inf
        self.point_recorded=self.image_recorded=-math.inf
        self.point_sequence=self.image_sequence=0
        self.consistency_reasons=[]
        self.last_observation=None
        self.last_decision=Decision()
        self.started=False
        self.nonce=secrets.token_hex(24)
        self.last_diagnostics=-math.inf

    def sensor(self,name,message):
        now=self.clock()
        try:
            self.monitor.ingest(name,message,now)
            record={'topic':name,'t':now,'message':jsonable(message)}
            if name=='ULIDAR_ARRAY':
                self.points=native_points(message)
                if now-self.point_recorded>=1.:
                    filename=f'cloud_{self.point_sequence:06d}.npz'
                    self.journal.put(filename,self.points)
                    record['npz_file']=filename
                    self.point_recorded=now
                    self.point_sequence+=1
            self.journal.put('samples.jsonl',record)
        except Exception as exc:
            reason='Ошибка пакета '+name+': '+type(exc).__name__
            self.monitor.faults.append(reason)
            self.journal.event('sensor_error',reason=reason)

    async def video(self,track):
        try:
            while not self.ending.is_set():
                frame=await track.recv()
                now=self.clock()
                source=float(frame.pts*frame.time_base) if frame.pts is not None and frame.time_base is not None else None
                self.monitor.ingest('CAMERA',{},now,source)
                # Декодирование вне цикла StopMove; храним только последний кадр.
                if now-self.image_received>=.2:
                    self.image=await asyncio.to_thread(frame.to_ndarray,format='bgr24')
                    self.image_received=now
                    if now-self.image_recorded>=1.:
                        filename=f'camera_{self.image_sequence:06d}.jpg'
                        self.journal.put(filename,self.image.copy())
                        self.journal.put('samples.jsonl',{'topic':'CAMERA','t':now,'source_s':source,'image_file':filename})
                        self.image_recorded=now
                        self.image_sequence+=1
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            if not self.ending.is_set():
                self.monitor.faults.append('Ошибка видео: '+type(exc).__name__)

    def health(self):
        now=self.clock()
        errors=self.monitor.health(now,self.policy,(*TOPICS[:3],'ULIDAR_ARRAY','CAMERA'))
        if self.journal.failed.is_set(): errors.append('Журнал переполнен или недоступен')
        if self.image is None or not 0<=now-self.image_received<=self.policy.max_observation_age:
            errors.append('Нет свежего декодированного изображения')
        errors+=self.consistency_reasons
        if not self.profile:
            errors.append('Не задан проверенный профиль робота')
        else:
            errors+=self.profile.session_errors(self.robot_id,self.monitor.identity,self.boot_id)
            for name in ('ROBOTODOM','ULIDAR_ARRAY'):
                age=self.monitor.calibrated_age(name,now,self.profile.time_contracts.get(name))
                if not -.03<=age<=self.policy.max_observation_age:
                    errors.append('Не подтверждена свежесть измерения: '+name)
            geometry=self.profile.geometry
            odom=self.monitor.streams.get('ROBOTODOM',Stream()).payload
            cloud=self.monitor.streams.get('ULIDAR_ARRAY',Stream()).payload
            if odom.get('header',{}).get('frame_id')!=geometry.get('pose_frame'):
                errors.append('Система координат позы не совпадает с профилем')
            if cloud.get('frame_id')!=geometry.get('point_frame'):
                errors.append('Система координат облака не совпадает с профилем')
            if geometry.get('source_kind')=='occupied_voxel_points' and cloud.get('resolution')!=geometry.get('voxel_size_m'):
                errors.append('Разрешение вокселей не совпадает с профилем')
        return list(dict.fromkeys(errors))

    async def observe(self):
        now=self.clock()
        pose=self.monitor.last_pose[1] if self.monitor.last_pose else None
        age=math.inf
        pose_age=math.inf
        contract={}
        if self.profile:
            age=self.monitor.calibrated_age('ULIDAR_ARRAY',now,self.profile.time_contracts.get('ULIDAR_ARRAY'))
            contract=self.profile.geometry_contract(self.robot_id,age)
            pose_age=self.monitor.calibrated_age('ROBOTODOM',now,self.profile.time_contracts.get('ROBOTODOM'))
            if self.profile.session_errors(self.robot_id,self.monitor.identity,self.boot_id):
                contract['identity_verified']=False
            odom=self.monitor.streams.get('ROBOTODOM',Stream()).payload
            cloud=self.monitor.streams.get('ULIDAR_ARRAY',Stream()).payload
            if odom.get('header',{}).get('frame_id')!=contract.get('pose_frame') or cloud.get('frame_id')!=contract.get('point_frame'):
                contract['geometry_frames_validated']=False
            if contract.get('source_kind')=='occupied_voxel_points' and cloud.get('resolution')!=contract.get('voxel_size_m'):
                contract['geometry_frames_validated']=False
        snapshot_epoch=self.monitor.pose_epoch
        sport=self.monitor.streams.get('LF_SPORT_MOD_STATE',Stream()).payload.copy()
        obs=await asyncio.to_thread(self.pipeline.build_observation,now,pose,self.points,
                                    snapshot_epoch,contract,self.image,self.policy)
        try:
            obs.roll,obs.pitch,_=sport['imu_state']['rpy']
            obs.body_height=sport['body_height']
            obs.speed=math.hypot(*sport['velocity'][:2])
        except (KeyError,ValueError,TypeError):
            obs.localized=False
            obs.diagnostics.append('Нет свежего полного состояния тела')
        # Медленный worker не может превратить старый кадр в свежий.
        if max(age,pose_age)+self.clock()-now>self.policy.max_observation_age or snapshot_epoch!=self.monitor.pose_epoch:
            obs.localized=False
            obs.diagnostics.append('Превышен бюджет обработки кадра')
        self.last_observation=obs
        return obs

    async def diagnostics(self):
        try:
            while not self.ending.is_set():
                if self.image is not None and self.monitor.last_pose and self.clock()-self.image_received<.6:
                    self.consistency_reasons=await asyncio.to_thread(self.consistency.observe,self.image,
                                              self.monitor.last_pose[1],self.image_received)
                await asyncio.sleep(.5)
        except asyncio.CancelledError: raise
        except Exception as exc:
            self.monitor.faults.append('Сбой диагностики: '+type(exc).__name__)

    def start_requested(self,path):
        """Учебный локальный сигнал. Судейский мост должен быть подключён отдельно."""
        if not path or not Path(path).is_file(): return False
        try:
            return Path(path).read_text(encoding='ascii').strip()==self.nonce
        except (OSError,UnicodeError): return False

    async def plan(self,duration,start_file,begin):
        deadline=self.clock()+duration
        while not self.ending.is_set() and self.clock()<deadline:
            obs=await self.observe()
            errors=self.health()
            if not self.started and self.live and self.start_requested(start_file):
                if errors:
                    self.journal.event('start_rejected',reasons=errors)
                    self.gate.trip('Сигнал старта при неподтверждённом состоянии')
                    self.ending.set()
                    break
                await begin()
                errors=self.health()
                if errors:
                    self.gate.trip('; '.join(errors))
                    self.ending.set()
                    break
                # После запроса режима старое наблюдение больше не используется.
                obs=await self.observe()
                if self.health() or not obs.localized:
                    self.gate.trip('Состояние изменилось перед стартом')
                    self.ending.set()
                    break
                self.started=True
                self.gate.submit(Decision(),self.clock())
                self.gate.arm()
            if self.started:
                if errors:
                    self.gate.trip('; '.join(errors))
                    self.ending.set()
                    break
                decision=self.mission.step(obs,start=self.mission.phase=='WAIT_START')
                self.last_decision=decision
                self.gate.submit(decision,self.clock())
                self.journal.event('decision',**asdict(decision))
                if decision.phase in ('FAULT','FINISHED'):
                    self.ending.set()
            if self.clock()-self.last_diagnostics>=1.:
                self.journal.event('diagnostic',health=errors,candidates=self.pipeline.candidates,
                                   observation=obs.diagnostics,consistency=self.consistency.report)
                self.last_diagnostics=self.clock()
            await asyncio.sleep(.05)
        self.ending.set()

async def run_session(args):
    """Вызывается CLI только с --connect/--execute; ни один тест не подключает Go2."""
    if sys.platform!='linux':
        raise RuntimeError('Бортовой запуск предназначен для Linux на назначенном Pi')
    profile=RobotProfile.load(args.profile) if args.profile else None
    live=args.mode=='live'
    if live:
        if not profile: raise ValueError('Live требует профиль конкретного робота')
        errors=profile.readiness(args.robot_id)
        if errors: raise ValueError('; '.join(errors))
        if Path(args.start_file).exists(): raise ValueError('Файл старта уже существует; старый сигнал запрещён')
        if Path(args.start_file).resolve().is_relative_to(Path(args.output).resolve()):
            raise ValueError('Сигнал старта должен находиться вне каталога создаваемого журнала')
    sdk=Path(args.sdk_dir).expanduser().resolve()
    if not (sdk/'go2.py').is_file(): raise FileNotFoundError('Нет штатного go2.py в --sdk-dir')
    # Файл блокирует второй экземпляр нашего клиента. fleet-dog управляется
    # по инструкции организатора, скрипт не останавливает чужие процессы.
    import fcntl
    lease=open('/tmp/wolf-go2-client.lock','a',encoding='ascii')
    fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    sys.path.insert(0,str(sdk))
    go2=importlib.import_module('go2')
    topics=importlib.import_module('unitree_webrtc_connect.constants').RTC_TOPIC
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    journal=Journal(args.output)
    runtime=Runtime(profile,args.robot_id,journal,boot_id,live)
    conn=None
    pump=None
    tasks=[]
    traffic=False
    loop=asyncio.get_running_loop()
    for sig in (signal.SIGINT,signal.SIGTERM): loop.add_signal_handler(sig,runtime.ending.set)
    try:
        conn=await asyncio.wait_for(go2.connect(retries=2,backoff=5),45.)
        conn.datachannel.set_decoder('native')
        for name in TOPICS:
            conn.datachannel.pub_sub.subscribe(topics[name],lambda message,n=name:runtime.sensor(n,message))
        conn.video.add_track_callback(runtime.video)
        conn.video.switchVideoChannel(True)
        traffic=bool(await asyncio.wait_for(conn.datachannel.disableTrafficSaving(True),5.))
        async def begin():
            await asyncio.wait_for(go2.ensure_normal_mode(conn),8.)
            reply=await asyncio.wait_for(conn.datachannel.pub_sub.publish_request_new(
                topics['MOTION_SWITCHER'],{'api_id':1001}),3.)
            data=reply.get('data',{}).get('data')
            if isinstance(data,str): data=json.loads(data)
            if not acknowledged(reply) or not isinstance(data,dict) or data.get('name')!='normal':
                raise RuntimeError('Normal mode не подтверждён штатным интерфейсом')
            if not await pump.stop('Исходная остановка перед разрешением движения',trip=False):
                raise RuntimeError('Исходная остановка не подтверждена')
        if live:
            pump=CommandPump(lambda command,param:go2.sport(conn,command,param),runtime.gate,
                             lambda:runtime.health() if runtime.gate.armed else [],journal.event)
            # Новая nonce действует только для этого экземпляра на этом Pi.
            (journal.path/'start-token.txt').write_text(runtime.nonce,encoding='ascii')
            journal.event('waiting_start',adapter='local_training_file_not_official_judge')
        tasks=[asyncio.create_task(runtime.diagnostics())]
        if pump: tasks.append(asyncio.create_task(pump.run(runtime.ending)))
        await runtime.plan(args.duration,args.start_file if live else None,begin)
    except Exception as exc:
        runtime.gate.trip(type(exc).__name__+': '+str(exc))
        journal.event('session_error',reason=runtime.gate.fault)
    finally:
        runtime.ending.set()
        substantive_fault=runtime.gate.fault
        if pump:
            # Это отдельная задача: долгий worker распознавания не задерживает StopMove.
            await pump.stop(runtime.gate.fault or 'Окончание сессии')
        for task in tasks: task.cancel()
        if tasks: await asyncio.gather(*tasks,return_exceptions=True)
        if conn:
            try:
                conn.video.switchVideoChannel(False)
                if traffic: await asyncio.wait_for(conn.datachannel.disableTrafficSaving(False),3.)
            except Exception: pass
            try: await asyncio.wait_for(go2.disconnect(conn),5.)
            except Exception as exc: journal.event('disconnect_error',error=type(exc).__name__)
        report={'scope':'onboard_training' if live else 'observation_only','started':runtime.started,
                'mission_phase':runtime.mission.phase,'completed':runtime.mission.completion_order,
                'fault':substantive_fault,'shutdown_reason':runtime.gate.fault,'sensor_report':runtime.monitor.report(runtime.clock()),
                'consistency':runtime.consistency.report,'stop_acknowledged':pump.stopped_ack if pump else None,
                'journal_dropped':journal.dropped,'journal_failed':journal.failed.is_set(),
                'physical_success_verified':False}
        journal.close()
        report['journal_failed']=journal.failed.is_set()
        (journal.path/'report.json').write_text(json.dumps(jsonable(report),ensure_ascii=False,indent=2),encoding='utf-8')
        lease.close()
    return report
