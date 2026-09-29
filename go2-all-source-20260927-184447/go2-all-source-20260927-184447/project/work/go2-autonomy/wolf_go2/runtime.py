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
from .sensors import SensorMonitor, Stream, stamp_seconds

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
        self.image_source=None
        self.image_metadata={}
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
                    # Номер кадра камеры не равен номеру облака. Сохраняем
                    # контекст приёма именно этого облака отдельным пакетом.
                    stem=filename[:-4]
                    pose_record=self.monitor.last_pose
                    pose_payload=self.monitor.streams.get('ROBOTODOM',Stream()).payload
                    camera_delta=now-self.image_received
                    paired_image=stem+'-camera.jpg' if self.image is not None and 0<=camera_delta<=self.policy.max_observation_age else None
                    if paired_image:
                        self.journal.put(paired_image,self.image.copy())
                    stream=self.monitor.streams['ULIDAR_ARRAY']
                    self.journal.put(stem+'.json',jsonable({
                        'schema_version':1,'cloud_file':filename,'received_at':now,
                        'pi_boot_id':self.boot_id,'frame_epoch':self.monitor.pose_epoch,
                        'cloud_source_s':stamp_seconds(stream.payload.get('stamp',stream.payload.get('header',{}).get('stamp'))),
                        'point_frame':stream.payload.get('frame_id'),
                        'resolution':stream.payload.get('resolution'),
                        'pose':asdict(pose_record[1]) if pose_record else None,
                        'pose_orientation_xyzw':pose_payload.get('pose',{}).get('orientation'),
                        'pose_source_header':pose_payload.get('header',{}),
                        'pose_received_at':pose_record[0] if pose_record else None,
                        'pose_receive_delta_s':now-pose_record[0] if pose_record else None,
                        'camera_file':paired_image,'camera_received_at':self.image_received if paired_image else None,
                        'camera_receive_delta_s':camera_delta if paired_image else None,
                        'camera_source_s':self.image_source if paired_image else None,
                        'camera_frame_metadata':dict(self.image_metadata) if paired_image else None,
                        'acquisition_alignment_verified':False,
                        'note':'Сопоставлено по приёму; задержка съёмки и калибровка проверяются отдельно'}))
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
                    self.image_metadata={
                        'pts':frame.pts,
                        'time_base':str(frame.time_base) if frame.time_base is not None else None,
                        'source_width':getattr(frame,'width',None),
                        'source_height':getattr(frame,'height',None),
                        'source_pixel_format':getattr(getattr(frame,'format',None),'name',None),
                        'decoded_width':int(self.image.shape[1]),
                        'decoded_height':int(self.image.shape[0]),
                        'runtime_spatial_transform':'none',
                        'calibration_verified':False}
                    self.image_received=now
                    self.image_source=source
                    if now-self.image_recorded>=1.:
                        filename=f'camera_{self.image_sequence:06d}.jpg'
                        self.journal.put(filename,self.image.copy())
                        self.journal.put('samples.jsonl',{'topic':'CAMERA','t':now,'source_s':source,'image_file':filename,
                                                        'frame_metadata':dict(self.image_metadata)})
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
        ready_deadline=self.clock()+35.
        while not self.ending.is_set() and self.clock()<deadline:
            obs=await self.observe()
            errors=self.health()
            button_start=getattr(self,'button_start',False)
            if button_start and not self.started and self.clock()>ready_deadline:
                self.gate.trip('Не достигнута готовность: '+('; '.join(errors) or 'нет свежего наблюдения'))
                self.ending.set()
                break
            if not self.started and self.live and (self.start_requested(start_file) or (button_start and not errors)):
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
                sink=getattr(self,'status_sink',None)
                if sink:
                    sink({'type':'status','phase':self.mission.phase,'started':self.started,
                          'health':errors,'reason':self.last_decision.reason if self.last_decision else '',
                          'completed':self.mission.completion_order})
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
    runtime.button_start=bool(getattr(args,'button_start',False))
    runtime.status_sink=getattr(args,'status_sink',None)
    if live and getattr(args,'trial_kind',None):
        from .training import TrainingController
        runtime.mission=TrainingController(runtime.policy,args.trial_kind,getattr(args,'stage_next',None),
            commissioning=bool(getattr(args,'commissioning',False)))
        journal.event('single_obstacle_trial',kind=args.trial_kind)
    conn=None
    pump=None
    tasks=[]
    traffic=False
    packet_audit=None
    wire_audit=None
    source_audit=None
    recovery=None
    loop=asyncio.get_running_loop()
    for sig in (signal.SIGINT,signal.SIGTERM): loop.add_signal_handler(sig,runtime.ending.set)
    try:
        if runtime.button_start or getattr(args,'console_control',False):
            async def watch_console():
                pipe=None
                try:
                    reader=asyncio.StreamReader()
                    protocol=asyncio.StreamReaderProtocol(reader)
                    pipe,_=await loop.connect_read_pipe(lambda:protocol,sys.stdin)
                    while not runtime.ending.is_set():
                        line=await reader.readline()
                        if not line or line.strip()==b'STOP':
                            runtime.gate.trip('Остановка оператором или закрытие канала панели')
                            runtime.ending.set()
                            return
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    runtime.gate.trip('Потерян канал остановки: '+type(exc).__name__)
                    runtime.ending.set()
                finally:
                    if pipe: pipe.close()
            tasks.append(asyncio.create_task(watch_console()))
        conn=await asyncio.wait_for(go2.connect(retries=2,backoff=5),45.)
        conn.datachannel.set_decoder('native')
        if not live:
            from .packet_audit import PacketAudit
            packet_audit=PacketAudit(conn.datachannel.decoder,journal)
            conn.datachannel.decoder=packet_audit
            from .wire_audit import WireAudit
            wire_decoder=getattr(conn.datachannel,'deal_array_buffer',None)
            if callable(wire_decoder):
                wire_audit=WireAudit(wire_decoder,journal)
                conn.datachannel.deal_array_buffer=wire_audit.decode
        for name in TOPICS:
            conn.datachannel.pub_sub.subscribe(topics[name],lambda message,n=name:runtime.sensor(n,message))
        if not live:
            from .source_audit import SourceAudit,SOURCES
            source_audit=SourceAudit()
            for name in SOURCES:
                if name in topics:
                    conn.datachannel.pub_sub.subscribe(topics[name],lambda message,n=name:source_audit.receive(n,message))
        conn.video.add_track_callback(runtime.video)
        conn.video.switchVideoChannel(True)
        traffic=bool(await asyncio.wait_for(conn.datachannel.disableTrafficSaving(True),5.))
        from .recovery import SensorRecovery
        recovery=SensorRecovery(runtime.clock())
        async def restore_sensor_transport():
            while not runtime.ending.is_set():
                received={name:runtime.monitor.streams.get(name,Stream()).received for name in TOPICS}
                received['CAMERA']=runtime.image_received
                for name in recovery.actions(runtime.clock(),received,runtime.started):
                    journal.event('sensor_transport_retry',topic=name,attempt=recovery.attempts[name])
                    try:
                        if name=='CAMERA':
                            conn.video.switchVideoChannel(False)
                            # Без await между off/on: старт миссии не может
                            # вклиниться и оставить камеру выключенной.
                            if not runtime.ending.is_set() and not runtime.started:
                                conn.video.switchVideoChannel(True)
                        else:
                            conn.datachannel.pub_sub.unsubscribe(topics[name])
                            conn.datachannel.pub_sub.subscribe(topics[name],lambda message,n=name:runtime.sensor(n,message))
                    except Exception as exc:
                        journal.event('sensor_transport_retry_failed',topic=name,error=type(exc).__name__)
                await asyncio.sleep(.5)
        tasks.append(asyncio.create_task(restore_sensor_transport()))
        async def begin():
            # Пользователь подтвердил допустимость Sport API в mcf 17.09.2026.
            # Разрешённый режим читаем без ненужного переключения прошивки.
            reply=await asyncio.wait_for(conn.datachannel.pub_sub.publish_request_new(
                topics['MOTION_SWITCHER'],{'api_id':1001}),3.)
            data=reply.get('data',{}).get('data')
            if isinstance(data,str): data=json.loads(data)
            if not acknowledged(reply) or not isinstance(data,dict) or data.get('name') not in ('normal','mcf'):
                raise RuntimeError('Режим normal/mcf не подтверждён штатным интерфейсом')
            if not await pump.stop('Исходная остановка перед разрешением движения',trip=False):
                raise RuntimeError('Исходная остановка не подтверждена')
        if live:
            pump=CommandPump(lambda command,param:go2.sport(conn,command,param),runtime.gate,
                             lambda:runtime.health() if runtime.gate.armed else [],journal.event)
            # Новая nonce действует только для этого экземпляра на этом Pi.
            (journal.path/'start-token.txt').write_text(runtime.nonce,encoding='ascii')
            journal.event('waiting_start',adapter='local_training_file_not_official_judge')
        tasks.append(asyncio.create_task(runtime.diagnostics()))
        if pump: tasks.append(asyncio.create_task(pump.run(runtime.ending)))
        await runtime.plan(args.duration,args.start_file if live else None,begin)
    except Exception as exc:
        runtime.gate.trip(type(exc).__name__+': '+str(exc))
        journal.event('session_error',reason=runtime.gate.fault)
    finally:
        runtime.ending.set()
        # Возраст пакетов измеряем до отключения; длительность закрытия WebRTC
        # не должна превращать исправный приём в ложную ошибку транспорта.
        packet_report=packet_audit.report() if packet_audit else None
        wire_report=wire_audit.report() if wire_audit else None
        source_report=source_audit.report() if source_audit else None
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
        if packet_report: report['packet_audit']=packet_report
        if wire_report: report['wire_audit']=wire_report
        if source_report: report['source_audit']=source_report
        if recovery: report['sensor_recovery']=recovery.report()
        if not live:
            from .scene_audit import inspect_monitor
            try:
                report['scene_audit']=inspect_monitor(runtime.points,runtime.monitor)
                if runtime.image is not None:
                    from .striped_poles import detect_striped_poles
                    report['scene_audit']['camera_striped_poles']=detect_striped_poles(runtime.image)
            except Exception as exc:
                report['scene_audit']={'scope':'diagnostic_only','route_verified':False,'reason':str(exc)}
            from .preparation import prepare_report
            report['preparation']=prepare_report(report,runtime.monitor,runtime.profile)
        report['journal_failed']=journal.failed.is_set()
        (journal.path/'report.json').write_text(json.dumps(jsonable(report),ensure_ascii=False,indent=2),encoding='utf-8')
        lease.close()
    return report
