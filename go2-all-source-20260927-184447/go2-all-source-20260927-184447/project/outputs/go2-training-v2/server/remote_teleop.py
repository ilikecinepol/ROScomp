"""Операторское управление Go2 через JSONL и запись датчиков на Raspberry Pi.

Подключается только с --execute. До свежего arm не меняет режим и не едет.
Записанная карта — обработанные воксели, не сырые лучи; координаты дрейфуют.
"""
from __future__ import annotations

import argparse
import asyncio
import base64
from collections import Counter, OrderedDict
from datetime import datetime, timezone
import io
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import shutil
import signal
import sys
import threading
import time

TOKEN_TTL = 0.35
HEARTBEAT_TTL = 0.35
SEND_TIMEOUT = 0.25
STOP_TIMEOUT = 0.45
MAX_SECONDS = 300
MAX_VX = 0.6
MAX_VY = 0.5
MAX_WZ = 1.0
PROFILES = {'precision': (0.10, 0.08, 0.20), 'obstacle': (0.15, 0.10, 0.25),
            'floor': (0.30, 0.15, 0.50), 'fast': (0.60, 0.50, 1.0)}


def json_safe(value):
    """Телеметрия полностью; большие массивы карты сохраняются отдельно."""
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    if isinstance(value, bytes):
        return {'binary_bytes': len(value)}
    if hasattr(value, 'shape') and hasattr(value, 'dtype'):
        return {'array_shape': list(value.shape), 'dtype': str(value.dtype)}
    if hasattr(value, 'item'):
        return json_safe(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if value is None or isinstance(value, (int, float, bool, str)):
        return value
    return str(value)


def acknowledged(response):
    try:
        return response['data']['header']['status']['code'] == 0
    except (KeyError, TypeError):
        return False


def confirmed_motion_mode(response):
    """Пользователь подтвердил допуск Sport Move в mcf 17.09.2026."""
    if not acknowledged(response):
        raise RuntimeError('Штатный режим не подтверждён')
    try:
        mode = json.loads(response['data']['data'])['name']
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError('Некорректный ответ о режиме движения') from exc
    if mode not in ('normal', 'mcf'):
        raise RuntimeError(f'Режим {mode!r} не разрешён: ожидается normal или mcf')
    return mode


def velocities(keys, profile='fast'):
    """Только шесть клавиш; внешние значения скорости не принимаются."""
    keys = set(keys)
    vx, vy, wz = PROFILES[profile]
    x = vx * (int('w' in keys) - int('s' in keys))
    y = vy * (int('q' in keys) - int('e' in keys))
    z = wz * (int('a' in keys) - int('d' in keys))
    # Диагональ не должна повышать полную поступательную скорость.
    speed = math.hypot(x, y)
    if speed > vx:
        x, y = x * vx / speed, y * vx / speed
    return {'x': x, 'y': y, 'z': z}


class Controller:
    """Логика допуска и независимый от получения команд watchdog."""
    def __init__(self, send, normal_mode, health, emit, clock=time.monotonic):
        self.send, self.normal_mode, self.health = send, normal_mode, health
        self.emit, self.clock = emit, clock
        self.tokens = OrderedDict()
        self.token = 0
        self.last_seq = -1
        self.last_input = None
        self.phase, self.reason = 'waiting_arm', 'Нажмите Arm для допуска движения'
        self.armed = False
        self.deadman = False
        self.keys = []
        self.epoch = 0
        self.closed = False
        self.move_task = None
        self.arm_task = None
        self.stop_lock = asyncio.Lock()
        self.next_move = 0.0
        self.was_moving = False
        self.final_stop_acknowledged = False
        self.counters = Counter()
        self.profile = 'precision'
        self.record_action = None

    def issue_token(self):
        self.token += 1
        self.tokens[self.token] = self.clock()
        while len(self.tokens) > 20:
            self.tokens.popitem(last=False)
        return self.token

    def log(self, event, **values):
        self.emit({'event': event, 'monotonic_s': self.clock(), **values})

    async def stop(self, reason, disarm=True):
        # Состояние сбрасывается до первого await: новые Move уже запрещены.
        if disarm:
            self.epoch += 1
            self.armed = False
            self.phase = 'stopped'
            self.deadman = False
            self.keys = []
            task = self.arm_task
            if task and task is not asyncio.current_task() and not task.done():
                task.cancel()
        self.reason = reason
        move = self.move_task
        if move and move is not asyncio.current_task() and not move.done():
            move.cancel()
        self.was_moving = False
        async with self.stop_lock:
            self.log('stop_requested', reason=reason, disarm=disarm)
            for attempt in range(1, 4):
                try:
                    self.log('command_sent', command='StopMove', attempt=attempt)
                    response = await asyncio.wait_for(self.send('StopMove', None), STOP_TIMEOUT)
                    self.log('command_response', command='StopMove', response=json_safe(response))
                    if acknowledged(response):
                        self.final_stop_acknowledged = True
                        self.counters['stop_acknowledged'] += 1
                        return True
                except Exception as exc:
                    self.log('command_error', command='StopMove', error=type(exc).__name__ + ': ' + str(exc)[:200])
            self.armed = False
            self.phase = 'error'
            self.reason = 'Нет подтверждения StopMove: оператор должен проверить остановку'
            self.counters['stop_unconfirmed'] += 1
            return False

    async def _arm(self, epoch):
        try:
            self.log('normal_mode_requested')
            await asyncio.wait_for(self.normal_mode(), 10.0)
            if epoch != self.epoch or self.closed:
                return
            if not await self.stop('Проверка StopMove перед допуском', disarm=False):
                return
            errors = self.health()
            if epoch != self.epoch or self.closed:
                return
            if errors or self.last_input is None or self.clock() - self.last_input > HEARTBEAT_TTL:
                await self.stop('Допуск отменён: ' + '; '.join(errors or ['нет свежего heartbeat']))
                return
            self.armed = True
            self.phase, self.reason = 'ready', 'Готов: движение только с удержанием deadman'
            self.deadman, self.keys = False, []
            self.log('armed')
        except asyncio.CancelledError:
            self.log('arming_cancelled')
            raise
        except Exception as exc:
            self.log('arm_error', error=type(exc).__name__ + ': ' + str(exc)[:200])
            await self.stop('Ошибка подготовки режима: ' + str(exc)[:180])

    async def handle(self, message):
        kind = message.get('type')
        if kind in ('stop', 'quit'):
            # Остановку принимаем даже при повторённом номере или старом token.
            if kind == 'quit':
                self.closed = True
            await self.stop('Закрытие оператором' if kind == 'quit' else str(message.get('reason') or 'Стоп оператором')[:200])
            return
        seq = message.get('seq')
        if not isinstance(seq, int) or isinstance(seq, bool) or seq <= self.last_seq:
            if kind in ('input', 'arm'):
                await self.stop('Повторённый или неверный номер команды')
            return
        self.last_seq = seq
        if kind == 'marker':
            self.log('marker', label=str(message.get('label', ''))[:160], seq=seq)
            return
        if kind not in ('arm', 'input', 'profile', 'record_start', 'record_stop'):
            return
        issued = self.tokens.get(message.get('token'))
        if issued is None or not 0 <= self.clock() - issued <= TOKEN_TTL:
            await self.stop('Просроченная команда: нужен новый arm')
            return
        if self.closed:
            return
        self.last_input = self.clock()
        self.log('operator_input', input=json_safe(message))
        if kind == 'profile':
            await self.stop('Смена скорости: отпустите клавиши')
            if message.get('profile') not in PROFILES:
                self.phase, self.reason = 'error', 'Неизвестный профиль скорости'
                return
            self.profile = message['profile']
            self.log('profile_changed', profile=self.profile)
            return
        if kind in ('record_start', 'record_stop'):
            await self.stop('Граница записи: отпустите клавиши')
            if self.record_action:
                self.record_action(kind, self.profile)
            return
        if kind == 'arm':
            if self.armed or (self.arm_task and not self.arm_task.done()):
                return
            errors = self.health()
            if errors:
                self.reason = 'Допуск запрещён: ' + '; '.join(errors)
                self.phase = 'waiting_arm'
                self.log('arm_rejected', errors=errors)
                return
            self.epoch += 1
            self.phase, self.reason = 'arming', 'Проверка штатного режима и остановки'
            self.arm_task = asyncio.create_task(self._arm(self.epoch))
            return
        keys = message.get('keys', [])
        if not isinstance(keys, list) or len(keys) > 6 or any(k not in ['w', 's', 'a', 'd', 'q', 'e'] for k in keys):
            await self.stop('Некорректные клавиши')
            return
        previous_deadman = self.deadman
        self.deadman = message.get('deadman') is True and self.armed
        self.keys = keys if self.deadman else []
        if (previous_deadman and not self.deadman) or (self.was_moving and not any(velocities(self.keys).values())):
            await self.stop('Клавиши отпущены', disarm=False)

    async def _move(self, parameter, epoch):
        try:
            # Повторная проверка непосредственно перед передачей в API.
            if not self.armed or not self.deadman or epoch != self.epoch or self.closed:
                return
            errors = self.health()
            if errors or self.last_input is None or self.clock() - self.last_input > HEARTBEAT_TTL:
                await self.stop('Перед Move потеряна свежесть: ' + '; '.join(errors))
                return
            self.was_moving = True
            self.final_stop_acknowledged = False
            self.counters['move_requests'] += 1
            self.log('command_sent', command='Move', parameter=parameter)
            response = await asyncio.wait_for(self.send('Move', parameter), SEND_TIMEOUT)
            self.log('command_response', command='Move', response=json_safe(response))
            if not acknowledged(response):
                await self.stop('Move не подтверждён')
        except asyncio.CancelledError:
            self.log('move_wait_cancelled')
            raise
        except Exception as exc:
            self.log('command_error', command='Move', error=type(exc).__name__ + ': ' + str(exc)[:200])
            await self.stop('Таймаут или ошибка Move')

    async def tick(self):
        now = self.clock()
        if self.armed or self.phase == 'arming':
            if self.last_input is None or now - self.last_input > HEARTBEAT_TTL:
                await self.stop('Потерян heartbeat: требуется повторный arm')
                return
            errors = self.health()
            if errors:
                await self.stop('Датчики или запись: ' + '; '.join(errors))
                return
        if self.armed and self.deadman and now >= self.next_move:
            parameter = velocities(self.keys, self.profile)
            if any(parameter.values()) and (self.move_task is None or self.move_task.done()):
                self.next_move = now + 0.10
                self.move_task = asyncio.create_task(self._move(parameter, self.epoch))

    async def eof(self):
        self.closed = True
        await self.stop('SSH stdin закрыт')


class StreamOutput:
    """Блокировка SSH stdout не останавливает watchdog в цикле asyncio."""
    def __init__(self, stream):
        self.fd = stream.fileno()
        self.queue = queue.Queue(maxsize=24)
        self.failed = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def put(self, message):
        try:
            self.queue.put_nowait(message)
        except queue.Full:
            self.failed.set()

    def _run(self):
        try:
            while True:
                item = self.queue.get()
                if item is None:
                    return
                data = (json.dumps(json_safe(item), ensure_ascii=False, separators=(',', ':')) + '\n').encode('utf-8')
                # os.write не держит блокировку Python stdout при обрыве VPN.
                offset = 0
                while offset < len(data):
                    offset += os.write(self.fd, data[offset:])
        except Exception:
            self.failed.set()


class Recorder:
    """Ограниченная очередь, запись и JPEG выполняются в отдельном потоке."""
    def __init__(self, out, preview):
        self.out, self.preview = out, preview
        self.started = time.monotonic()
        self.queue = queue.Queue(maxsize=160)
        self.failed = threading.Event()
        self.error = None
        self.counts = Counter()
        self.latest = {}
        self.stamps = {}
        self.camera_at = None
        self.camera_pts = None
        self.camera_progress_at = None
        self.image_counter = 0
        self.map_counter = 0
        self.last_map = -100.0
        self.last_image = -100.0
        self.last_preview = -100.0
        self.active_recording = None
        self.last_recording = None
        self.saved_recording = None
        self.record_number = 0
        self.saved_images = 0
        self.saved_maps = 0
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def put(self, kind, record, payload=None):
        if kind in ('telemetry', 'maps') and self.active_recording is None:
            return
        record = dict(record)
        record['_recording'] = self.active_recording
        try:
            self.queue.put_nowait((kind, record, payload))
        except queue.Full:
            self.error = 'Очередь записи переполнена'
            self.failed.set()

    def recording(self, action, profile):
        if action == 'record_start' and self.active_recording is None:
            self.record_number += 1
            self.active_recording = 'run_%03d' % self.record_number
            self.last_recording = self.active_recording
            self.event({'event': 'record_start', 'profile': profile,
                        'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                        'time_utc': datetime.now(timezone.utc).isoformat()})
        elif action == 'record_stop' and self.active_recording is not None:
            self.event({'event': 'record_stop', 'time_utc': datetime.now(timezone.utc).isoformat()})
            self.put('finish', {'profile': profile})
            self.active_recording = None

    def event(self, item):
        self.put('events', {'elapsed_s': time.monotonic() - self.started, **json_safe(item)})

    def callback(self, topic):
        def receive(message):
            now = time.monotonic()
            data = message.get('data', {})
            self.latest[topic] = (now, data)
            self.counts[topic] += 1
            stamp = data.get('stamp') if topic == 'LF_SPORT_MOD_STATE' else data.get('header', {}).get('stamp')
            if stamp is not None:
                encoded = json.dumps(json_safe(stamp), sort_keys=True)
                old = self.stamps.get(topic)
                if old is None or old[0] != encoded:
                    self.stamps[topic] = (encoded, now)
            self.put('telemetry', {'topic': topic, 'elapsed_s': now - self.started,
                                  'receive_monotonic_s': now, 'message': json_safe(message)})
        return receive

    def health(self):
        now, errors = time.monotonic(), []
        if self.failed.is_set():
            errors.append(self.error or 'Ошибка записи')
        for topic, maximum in [('LF_SPORT_MOD_STATE', 0.5), ('ROBOTODOM', 0.5), ('LOW_STATE', 2.0)]:
            if topic not in self.latest or now - self.latest[topic][0] > maximum:
                errors.append('Нет свежего ' + topic)
        for topic in ['LF_SPORT_MOD_STATE', 'ROBOTODOM']:
            if topic not in self.stamps or now - self.stamps[topic][1] > 0.5:
                errors.append('Не обновляется stamp ' + topic)
        if self.camera_at is None or now - self.camera_at > 1.0:
            errors.append('Нет свежей камеры')
        if self.camera_progress_at is None or now - self.camera_progress_at > 1.0:
            errors.append('Не обновляется PTS камеры')
        try:
            sport = self.latest['LF_SPORT_MOD_STATE'][1]
            r, p = sport['imu_state']['rpy'][:2]
            if not all(math.isfinite(v) for v in (r, p)) or abs(r) > math.radians(35) or abs(p) > math.radians(40):
                errors.append('Наклон больше 35° крена / 40° тангажа')
            if not 0.12 < float(sport['body_height']) < 0.65:
                errors.append('Высота корпуса вне рабочего диапазона')
        except (KeyError, TypeError, ValueError):
            errors.append('Нет полного IMU/состояния корпуса')
        return errors

    def _run(self):
        files = {}
        try:
            for name in ['events', 'telemetry', 'camera', 'maps']:
                files[name] = (self.out / (name + '.jsonl')).open('w', encoding='utf-8')
            (self.out / 'camera').mkdir(exist_ok=True)
            (self.out / 'maps').mkdir(exist_ok=True)
            disk_checked = -100.0
            while True:
                if time.monotonic() - disk_checked > 1:
                    disk_checked = time.monotonic()
                    if shutil.disk_usage(self.out).free < 250 * 1024 * 1024:
                        raise OSError('Свободно менее 250 МБ')
                try:
                    item = self.queue.get(timeout=1)
                except queue.Empty:
                    continue
                if item is None:
                    return
                kind, record, payload = item
                run_name = record.pop('_recording', None)
                target = self.out / run_name if run_name else self.out
                if run_name:
                    target.mkdir(exist_ok=True)
                    for sub in ('camera', 'maps'):
                        (target / sub).mkdir(exist_ok=True)
                if kind == 'finish':
                    for key in list(files):
                        if key.startswith(run_name + '/'):
                            files.pop(key).close()
                    (target / 'recording.json').write_text(json.dumps({'complete': True,
                        'recording': run_name, **record}, ensure_ascii=False), encoding='utf-8')
                    self.saved_recording = run_name
                    continue
                if kind == 'camera':
                    picture = payload.to_image()
                    if run_name:
                        picture.save(target / record['file'], quality=85)
                        self.saved_images += 1
                    # Очередь записи не должна показывать старый кадр как свежий.
                    if record.pop('preview', False) and time.monotonic() - record['receive_monotonic_s'] <= 0.6:
                        picture.thumbnail((640, 360))
                        buffer = io.BytesIO()
                        picture.save(buffer, format='JPEG', quality=70)
                        self.preview.put({'type': 'frame', 'seq': record['seq'], 'elapsed_s': record['elapsed_s'],
                                          'jpeg': base64.b64encode(buffer.getvalue()).decode('ascii')})
                elif kind == 'maps':
                    (target / record['file']).write_bytes(payload)
                    self.saved_maps += 1
                if run_name:
                    key = run_name + '/' + kind
                    if key not in files:
                        files[key] = (target / (kind + '.jsonl')).open('a', encoding='utf-8')
                    files[key].write(json.dumps(record, ensure_ascii=False) + '\n')
                    files[key].flush()
                if kind == 'events':
                    files[kind].write(json.dumps(record, ensure_ascii=False) + '\n')
                    files[kind].flush()
        except Exception as exc:
            self.error = type(exc).__name__ + ': ' + str(exc)[:200]
            self.failed.set()
        finally:
            for stream in files.values():
                stream.close()


class MapCapture:
    """Копирует до 5 Гц сжатых пакетов и делегирует штатному native decoder."""
    def __init__(self, delegate, recorder):
        self.delegate, self.rec = delegate, recorder

    def decode(self, binary, metadata):
        now = time.monotonic()
        if 'src_size' in metadata and now - self.rec.last_map >= 0.2:
            self.rec.last_map = now
            number = self.rec.map_counter
            self.rec.map_counter += 1
            self.rec.put('maps', {'seq': number, 'elapsed_s': now - self.rec.started,
                'receive_monotonic_s': now, 'file': f'maps/map_{number:06d}.bin',
                'semantics': 'compressed_voxel_map_not_raw_lidar', 'metadata': json_safe(metadata)}, bytes(binary))
        return self.delegate.decode(binary, metadata)


async def run(out, output):
    sys.path.insert(0, str(Path.home() / 'ai-robot'))
    import go2
    from unitree_webrtc_connect.constants import RTC_TOPIC
    rec = Recorder(out, output)
    conn = None
    ctl = None
    traffic_changed = False
    ending = asyncio.Event()
    video_tasks = set()
    reader_task = None
    loop = asyncio.get_running_loop()
    errors = []
    for signum in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(signum, ending.set)

    async def video(track):
        task = asyncio.current_task()
        video_tasks.add(task)
        try:
            while not ending.is_set():
                frame = await track.recv()
                now = time.monotonic()
                rec.camera_at = now
                rec.counts['CAMERA'] += 1
                if frame.pts != rec.camera_pts:
                    rec.camera_progress_at, rec.camera_pts = now, frame.pts
                if now - rec.last_image >= 0.2:
                    preview = now - rec.last_preview >= 0.5
                    if preview:
                        rec.last_preview = now
                    rec.last_image = now
                    number = rec.image_counter
                    rec.image_counter += 1
                    rec.put('camera', {'seq': number, 'elapsed_s': now - rec.started,
                        'receive_monotonic_s': now, 'pts': frame.pts, 'time_base': str(frame.time_base),
                        'width': frame.width, 'height': frame.height,
                        'file': f'camera/frame_{number:06d}.jpg', 'preview': preview}, frame)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            errors.append('Камера: ' + type(exc).__name__)
            rec.event({'event': 'video_error', 'error': str(exc)[:200]})
        finally:
            video_tasks.discard(task)

    try:
        conn = await asyncio.wait_for(go2.connect(retries=2, backoff=5), 40)
        conn.datachannel.set_decoder('native')
        conn.datachannel.decoder = MapCapture(conn.datachannel.decoder, rec)
        for topic in ['LF_SPORT_MOD_STATE', 'LOW_STATE', 'ROBOTODOM', 'ULIDAR_STATE', 'ULIDAR_ARRAY']:
            conn.datachannel.pub_sub.subscribe(RTC_TOPIC[topic], rec.callback(topic))
        conn.video.add_track_callback(video)
        conn.video.switchVideoChannel(True)
        try:
            traffic_changed = bool(await asyncio.wait_for(conn.datachannel.disableTrafficSaving(True), 5))
        except Exception as exc:
            rec.event({'event': 'lidar_enable_error', 'error': str(exc)[:200]})

        async def send(command, parameter):
            return await go2.sport(conn, command, parameter)

        async def normal_mode():
            # mcf разрешён: читаем режим, без ненужного переключения прошивки.
            response = await conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC['MOTION_SWITCHER'], {'api_id': 1001})
            mode = confirmed_motion_mode(response)
            rec.event({'event': 'motion_mode_verified', 'mode': mode})

        def health():
            reasons = rec.health()
            if output.failed.is_set():
                reasons.append('SSH вывод недоступен или переполнен')
            return reasons

        ctl = Controller(send, normal_mode, health, rec.event)
        ctl.record_action = rec.recording
        reader = asyncio.StreamReader(limit=16384)
        protocol = asyncio.StreamReaderProtocol(reader)
        await loop.connect_read_pipe(lambda: protocol, sys.stdin)

        async def input_loop():
            try:
                while not ending.is_set():
                    line = await reader.readline()
                    if not line:
                        await ctl.eof()
                        ending.set()
                        return
                    try:
                        message = json.loads(line)
                        if not isinstance(message, dict):
                            raise ValueError('JSON должен быть объектом')
                        await ctl.handle(message)
                    except (ValueError, TypeError) as exc:
                        rec.event({'event': 'protocol_error', 'error': str(exc)[:200]})
                        await ctl.stop('Ошибка протокола оператора')
                    if ctl.closed:
                        ending.set()
            except Exception as exc:
                rec.event({'event': 'stdin_error', 'error': str(exc)[:200]})
                await ctl.eof()
                ending.set()

        reader_task = asyncio.create_task(input_loop())
        last_status = -100.0
        while not ending.is_set() and time.monotonic() - rec.started < MAX_SECONDS:
            await ctl.tick()
            now = time.monotonic()
            if now - last_status >= 0.1:
                last_status = now
                sport = rec.latest.get('LF_SPORT_MOD_STATE')
                output.put({'type': 'status', 'phase': ctl.phase, 'token': ctl.issue_token(),
                    'protocol_version': 2, 'profile': ctl.profile,
                    'recording': rec.active_recording, 'saved_recording': rec.saved_recording,
                    'telemetry': json_safe(sport[1]) if sport else {},
                    'armed': ctl.armed, 'reason': ctl.reason,
                    'telemetry_age_s': None if sport is None else now - sport[0],
                    'camera_age_s': None if rec.camera_at is None else now - rec.camera_at,
                    'health_errors': health(), 'elapsed_s': now - rec.started,
                    'remaining_s': max(0, MAX_SECONDS - (now - rec.started)),
                    'session_dir': str(out), 'recorded_frames': rec.saved_images,
                    'recorded_maps': rec.saved_maps})
            if output.failed.is_set() or rec.failed.is_set():
                await ctl.stop('Остановка из-за ошибки канала или записи')
                ending.set()
            await asyncio.sleep(0.02)
    except Exception as exc:
        errors.append(type(exc).__name__ + ': ' + str(exc)[:300])
        rec.event({'event': 'fatal_error', 'error': errors[-1]})
        output.put({'type': 'status', 'phase': 'error', 'armed': False, 'reason': errors[-1], 'session_dir': str(out)})
    finally:
        ending.set()
        if ctl:
            ctl.closed = True
            await ctl.stop('Завершение записи / предел 300 секунд')
        elif conn is not None:
            try:
                await asyncio.wait_for(go2.sport(conn, 'StopMove', None), STOP_TIMEOUT)
            except Exception as exc:
                errors.append('Финальный StopMove: ' + type(exc).__name__)
        if reader_task:
            reader_task.cancel()
            await asyncio.gather(reader_task, return_exceptions=True)
        if conn is not None:
            try:
                conn.video.switchVideoChannel(False)
                if traffic_changed:
                    await asyncio.wait_for(conn.datachannel.disableTrafficSaving(False), 3)
            except Exception as exc:
                errors.append('Выключение потоков: ' + type(exc).__name__)
            try:
                await asyncio.wait_for(go2.disconnect(conn), 5)
            except Exception as exc:
                errors.append('Отключение: ' + type(exc).__name__)
        for task in list(video_tasks):
            task.cancel()
        await asyncio.gather(*video_tasks, return_exceptions=True)
        rec.recording('record_stop', ctl.profile if ctl else 'precision')
        try:
            rec.queue.put_nowait(None)
        except queue.Full:
            errors.append('Очередь не завершена: переполнение')
        await asyncio.to_thread(rec.thread.join, 8)
        if rec.thread.is_alive():
            errors.append('Запись не завершилась за 8 секунд')
        summary = {'time_utc': datetime.now(timezone.utc).isoformat(), 'elapsed_s': time.monotonic() - rec.started,
            'purpose': 'operator_training_recording_not_autonomous_replay', 'session_dir': str(out),
            'limits': {'vx_m_s': MAX_VX, 'vy_m_s': MAX_VY, 'wz_rad_s': MAX_WZ,
                       'translation_norm_m_s': MAX_VX, 'heartbeat_s': HEARTBEAT_TTL, 'max_seconds': MAX_SECONDS},
            'lidar_semantics': 'compressed_voxel_map_not_raw_lidar',
            'camera_timestamp_semantics': 'receive_monotonic_and_video_pts_not_hardware_synced',
            'counts': dict(rec.counts), 'recorded_frames': rec.saved_images, 'recorded_maps': rec.saved_maps,
            'commands': dict(ctl.counters) if ctl else {},
            'final_stop_acknowledged': bool(ctl and ctl.final_stop_acknowledged),
            'writer_error': rec.error, 'errors': errors}
        (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        output.put({'type': 'summary', **summary})
    return 0 if not errors and not rec.failed.is_set() and ctl and ctl.final_stop_acknowledged else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps({'dry_run': True, 'connects': False, 'needs_explicit_arm': True,
                          'limits': {'vx': MAX_VX, 'vy': MAX_VY, 'wz': MAX_WZ, 'seconds': MAX_SECONDS}}))
        return 0
    args.output.mkdir(parents=True, exist_ok=True)
    if any((args.output / name).exists() for name in ['summary.json', 'telemetry.jsonl', 'events.jsonl', 'camera', 'maps']):
        parser.error('Каталог уже содержит запись: укажите новый каталог сеанса')
    original_stdout = sys.stdout
    # Сообщения сторонних библиотек не должны ломать JSONL протокол.
    sys.stdout = sys.stderr
    output = StreamOutput(original_stdout)
    code = asyncio.run(run(args.output, output))
    try:
        output.queue.put_nowait(None)
        output.thread.join(timeout=2)
    except queue.Full:
        pass
    return code


if __name__ == '__main__':
    raise SystemExit(main())
