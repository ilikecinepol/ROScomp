"""Короткая проверка поворота: 0,5 рад/с, не более одной секунды."""
from __future__ import annotations
import argparse
import asyncio
import json
import math
import secrets
import signal
import sys
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

VX, DURATION = 0.0, 1.0


def ack_code(response):
    try:
        return response['data']['header']['status']['code']
    except (KeyError, TypeError):
        return None


async def run_pulse(send, health, emit, abort):
    """Таймер StopMove работает отдельно от ожидания подтверждений Move."""
    started = time.monotonic()
    deadline = started + DURATION
    stopped = False
    lock = asyncio.Lock()
    state = {'move_attempts': 0, 'stop_acknowledged': False, 'stop_reason': None}

    async def stop(reason):
        nonlocal stopped
        abort.set()
        async with lock:
            if stopped:
                return
            state['stop_reason'] = reason
            for attempt in range(3):
                try:
                    emit('command', command='StopMove', reason=reason, attempt=attempt + 1)
                except Exception:
                    pass  # Ошибка журнала не должна препятствовать остановке.
                try:
                    response = await asyncio.wait_for(send('StopMove', None), timeout=0.8)
                    if ack_code(response) == 0:
                        state['stop_acknowledged'] = True
                        stopped = True
                    try:
                        emit('response', command='StopMove', response=response)
                    except Exception:
                        pass
                    if stopped:
                        return
                except Exception as exc:
                    try:
                        emit('command_error', command='StopMove', error=type(exc).__name__)
                    except Exception:
                        pass
            stopped = True

    async def watchdog():
        while not abort.is_set():
            reasons = health()
            if reasons:
                await stop('sensor_guard: ' + '; '.join(reasons))
                return
            if time.monotonic() >= deadline:
                await stop('bounded_duration_limit')
                return
            await asyncio.sleep(0.02)
        await stop('abort_requested')

    watcher = asyncio.create_task(watchdog())
    try:
        while not abort.is_set() and time.monotonic() < deadline:
            reasons = health()
            if reasons:
                state['stop_reason'] = 'sensor_guard: ' + '; '.join(reasons)
                break
            # Проверяем срок непосредственно перед каждой отправкой.
            parameter = {'x': VX, 'y': 0.0, 'z': 0.5}
            emit('command', command='Move', parameter=parameter)
            if abort.is_set() or time.monotonic() >= deadline:
                break
            state['move_attempts'] += 1
            try:
                response = await asyncio.wait_for(send('Move', parameter), timeout=0.3)
                emit('response', command='Move', response=response)
                if ack_code(response) != 0:
                    state['stop_reason'] = 'move_not_acknowledged'
                    break
            except Exception as exc:
                emit('command_error', command='Move', error=type(exc).__name__)
                state['stop_reason'] = 'move_response_error'
                break
            await asyncio.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
    finally:
        await stop(state['stop_reason'] or 'bounded_duration_limit')
        await watcher
    state['pulse_elapsed_s'] = time.monotonic() - started
    return state


class Recorder:
    def __init__(self, out):
        self.out = out
        self.start = time.monotonic()
        self.phase = 'preflight'
        self.latest = {}
        self.stamps = {}
        self.log_fault = False
        self.counts = Counter()
        self.camera_records = []
        self.errors = []
        self.latest_camera = None
        self.reference_position = None
        self.reference_error = None
        self.samples = (out/'samples.jsonl').open('w', encoding='utf-8')
        self.events = (out/'events.jsonl').open('w', encoding='utf-8')

    def emit(self, event, **values):
        try:
            self.events.write(json.dumps({'event': event, 't_s': time.monotonic()-self.start,
                                         'phase': self.phase, **values}, ensure_ascii=False) + '\n')
            self.events.flush()
        except Exception:
            self.log_fault = True

    def callback(self, topic):
        def receive(message):
            now = time.monotonic()
            data = message.get('data', {})
            self.latest[topic] = (now, data)
            stamp = data.get('stamp') if topic == 'LF_SPORT_MOD_STATE' else data.get('header', {}).get('stamp')
            if stamp is not None:
                token = json.dumps(stamp, sort_keys=True)
                previous = self.stamps.get(topic)
                if previous is None or previous[0] != token:
                    self.stamps[topic] = (token, now)
            self.counts[topic] += 1
            try:
                self.samples.write(json.dumps({'topic': topic, 'elapsed_s': now-self.start,
                    'phase': self.phase, 'message': message}, ensure_ascii=False) + '\n')
                self.samples.flush()
            except Exception:
                self.log_fault = True
        return receive

    def health(self):
        now, errors = time.monotonic(), []
        if self.log_fault:
            errors.append('ошибка записи журнала')
        for topic in ['LOW_STATE', 'LF_SPORT_MOD_STATE', 'ROBOTODOM']:
            # LOW_STATE на текущем соединении приходит около 1 Гц.
            # Быстрый контроль наклона/скорости остаётся по SPORT с пределом 0,5 с.
            max_age = 2.0 if topic == 'LOW_STATE' else 0.5
            if topic not in self.latest or now-self.latest[topic][0] > max_age:
                errors.append('нет свежего ' + topic)
        for topic in ['LF_SPORT_MOD_STATE', 'ROBOTODOM']:
            if topic not in self.stamps or now-self.stamps[topic][1] > 0.5:
                errors.append('не обновляется stamp ' + topic)
        if self.latest_camera is None or now-self.latest_camera > 1.5:
            errors.append('нет свежей камеры')
        if errors:
            return errors
        low = self.latest['LOW_STATE'][1]
        sport = self.latest['LF_SPORT_MOD_STATE'][1]
        try:
            for imu in [low['imu_state'], sport['imu_state']]:
                r, p = imu['rpy'][:2]
                if not all(math.isfinite(v) for v in [r, p]) or max(abs(r), abs(p)) > math.radians(15):
                    errors.append('наклон превышает 15 градусов')
            h = float(sport['body_height'])
            if not math.isfinite(h) or not 0.22 <= h <= 0.5:
                errors.append('высота корпуса вне диапазона теста')
            soc = low['bms_state']['soc']
            if not math.isfinite(soc) or soc < 20:
                errors.append('заряд меньше 20 процентов')
            temperatures = [float(m['temperature']) for m in low['motor_state'][:12]]
            if len(temperatures) != 12 or not all(math.isfinite(v) and v <= 60 for v in temperatures):
                errors.append('температура моторов вне диапазона теста')
            velocity = sport['velocity'][:2]
            if len(velocity) != 2 or not all(math.isfinite(v) for v in velocity) or math.hypot(*velocity) > 0.25:
                errors.append('сообщаемая скорость выше 0,25 м/с')
            pos = sport['position'][:2]
            if len(pos) != 2 or not all(math.isfinite(v) for v in pos):
                errors.append('некорректная сообщаемая позиция')
            elif self.reference_position is not None and math.dist(pos, self.reference_position) > 0.20:
                errors.append('сообщаемое смещение больше 20 см')
            if self.reference_error is not None and sport.get('error_code') != self.reference_error:
                errors.append('изменился error_code')
        except (KeyError, TypeError, ValueError):
            errors.append('неполные данные состояния')
        return errors


async def main(out):
    root = Path.home()/'ai-robot'
    sys.path.insert(0, str(root))
    import go2
    from unitree_webrtc_connect.constants import RTC_TOPIC
    rec = Recorder(out)
    abort = asyncio.Event()
    ending = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in [signal.SIGINT, signal.SIGTERM]:
        loop.add_signal_handler(sig, abort.set)
    conn = None
    video_task = None
    motion_possible = False
    result = {'executed': False, 'time_utc': datetime.now(timezone.utc).isoformat(),
              'plan': {'vx_m_s': VX, 'vy_m_s': 0, 'vyaw_rad_s': 0.5, 'duration_s': DURATION},
              'errors': rec.errors}

    async def video(track):
        nonlocal video_task
        video_task = asyncio.current_task()
        last = -100.0
        while not ending.is_set():
            try:
                frame = await track.recv()
                now = time.monotonic()
                rec.latest_camera = now
                rec.counts['CAMERA'] += 1
                if now-last >= 0.25:
                    name = f'camera_{len(rec.camera_records):03d}.jpg'
                    phase = rec.phase
                    item = {'file': name, 'elapsed_s': now-rec.start, 'phase': phase,
                            'pts': frame.pts, 'time_base': str(frame.time_base)}
                    await asyncio.to_thread(lambda: frame.to_image().save(out/name, quality=88))
                    rec.camera_records.append(item)
                    last = now
            except Exception as exc:
                if not ending.is_set():
                    rec.errors.append('video: ' + type(exc).__name__)
                    abort.set()
                return

    async def send(command, parameter):
        if command == 'Move' and abort.is_set():
            raise RuntimeError('Move отменён ограничителем')
        return await go2.sport(conn, command, parameter)

    try:
        conn = await asyncio.wait_for(go2.connect(retries=2, backoff=5), timeout=40)
        conn.video.add_track_callback(video)
        for topic in ['LOW_STATE', 'LF_SPORT_MOD_STATE', 'ROBOTODOM']:
            conn.datachannel.pub_sub.subscribe(RTC_TOPIC[topic], rec.callback(topic))
        conn.video.switchVideoChannel(True)
        # Сначала снимок и проверка состояния; движения до arm.json нет.
        limit = time.monotonic()+10
        while (rec.health() or not rec.camera_records) and time.monotonic() < limit and not abort.is_set():
            await asyncio.sleep(0.1)
        if rec.health() or not rec.camera_records or abort.is_set():
            raise RuntimeError('preflight: ' + '; '.join(rec.health()))
        nonce = secrets.token_hex(12)
        ready = {'nonce': nonce, 'ready_at_utc': datetime.now(timezone.utc).isoformat(),
                 'expires_in_s': 120, 'camera_file': rec.camera_records[-1]['file'],
                 'health_errors': rec.health(), 'plan': result['plan']}
        (out/'ready.json').write_text(json.dumps(ready, ensure_ascii=False, indent=2), encoding='utf-8')
        rec.emit('ready', **ready)
        deadline = time.monotonic()+120
        armed = False
        while time.monotonic() < deadline and not abort.is_set():
            arm = out/'arm.json'
            if arm.exists():
                value = json.loads(arm.read_text())
                if value.get('nonce') != nonce or value.get('operator_ready') is not True:
                    raise RuntimeError('arm.json не соответствует текущему тесту')
                armed = True
                break
            await asyncio.sleep(0.1)
        if not armed:
            raise RuntimeError('Тест не разрешён за время ожидания')
        rec.emit('armed')
        if rec.health():
            raise RuntimeError('Состояние изменилось до старта: ' + '; '.join(rec.health()))
        rec.phase = 'mode_setup'
        rec.emit('mode_check_start')
        reply = await asyncio.wait_for(conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC['MOTION_SWITCHER'], {'api_id': 1001}), timeout=3)
        mode = json.loads(reply['data']['data']).get('name') if ack_code(reply) == 0 else None
        if mode not in ('normal', 'mcf'):
            raise RuntimeError('Не подтверждён разрешённый режим normal/mcf')
        rec.emit('mode_verified', mode=mode)
        # Проверяем приём штатной остановки до первого Move.
        response = await asyncio.wait_for(send('StopMove', None), timeout=0.8)
        rec.emit('response', command='StopMove', response=response, reason='before_test')
        if ack_code(response) != 0:
            raise RuntimeError('Нет подтверждения StopMove до теста')
        rec.phase = 'before'
        rec.emit('phase_start')
        for _ in range(30):
            if abort.is_set() or rec.health():
                raise RuntimeError('baseline: ' + '; '.join(rec.health()))
            await asyncio.sleep(0.1)
        rec.reference_position = list(rec.latest['LF_SPORT_MOD_STATE'][1]['position'][:2])
        rec.reference_error = rec.latest['LF_SPORT_MOD_STATE'][1].get('error_code')
        rec.phase = 'move'
        rec.emit('phase_start')
        motion_possible = True
        result['pulse'] = await run_pulse(send, rec.health, rec.emit, abort)
        result['executed'] = result['pulse']['move_attempts'] > 0
        if not result['pulse']['stop_acknowledged']:
            raise RuntimeError('Нет подтверждения остановки; требуется проверка оператором')
        rec.phase = 'after'
        rec.emit('phase_start')
        await asyncio.sleep(5)
    except Exception as exc:
        rec.errors.append(type(exc).__name__ + ': ' + str(exc))
        rec.emit('test_error', error=rec.errors[-1])
    finally:
        abort.set()
        if conn is not None:
            if motion_possible:
                # Повторная остановка при любой ошибке после начала попытки.
                for attempt in range(3):
                    try:
                        rec.emit('command', command='StopMove', reason='final_cleanup', attempt=attempt+1)
                        response = await asyncio.wait_for(send('StopMove', None), timeout=0.8)
                        rec.emit('response', command='StopMove', response=response, reason='final_cleanup')
                        if ack_code(response) == 0:
                            result['final_stop_acknowledged'] = True
                            break
                    except Exception as exc:
                        rec.emit('command_error', command='StopMove', error=type(exc).__name__)
            ending.set()
            try:
                conn.video.switchVideoChannel(False)
                await asyncio.wait_for(go2.disconnect(conn), timeout=5)
            except Exception as exc:
                rec.errors.append('disconnect: ' + type(exc).__name__)
            if video_task and not video_task.done():
                video_task.cancel()
                await asyncio.gather(video_task, return_exceptions=True)
        result.update({'counts': dict(rec.counts), 'camera_records': rec.camera_records,
                       'final_health_errors': rec.health()})
        (out/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        rec.samples.close()
        rec.events.close()
    print(json.dumps({k:v for k,v in result.items() if k!='camera_records'}, ensure_ascii=False))
    return 0 if result.get('executed') and result.get('final_stop_acknowledged') and not rec.errors else 2


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps({'dry_run': True, 'vy': 0.0, 'vx': VX, 'duration': DURATION, 'turn': 0.5,
                          'note': 'Соединения нет. Для живого теста нужны --execute и свежий arm.json.'}, ensure_ascii=False))
        raise SystemExit(0)
    args.output.mkdir(parents=True, exist_ok=True)
    raise SystemExit(asyncio.run(main(args.output)))
