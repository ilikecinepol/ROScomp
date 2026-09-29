#!/usr/bin/env python3
"""Окно ручного управления Go2: клавиатура, камера и отметки в записи.

Подготовка автоматическая; движение только при удержании клавиши направления.
Проверка без робота: python keyboard_client.py --dry-run.
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
import queue
import re
import shutil
import subprocess
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[2]
KEYS = frozenset("wsadqe")
PHYSICAL_KEYS = {87: "w", 83: "s", 65: "a", 68: "d", 81: "q", 69: "e", 32: "space", 88: "x"}
RUSSIAN_KEYS = {"ц": "w", "ы": "s", "ф": "a", "в": "d", "й": "q", "у": "e", "ч": "x"}


def key_name(keysym: str, keycode: int = 0) -> str:
    """На Windows распознаём физические клавиши независимо от раскладки."""
    return PHYSICAL_KEYS.get(keycode, RUSSIAN_KEYS.get(keysym.lower(), keysym.lower()))


def finite_age(value: object, limit: float) -> bool:
    try:
        number = float(value)
        return math.isfinite(number) and 0 <= number <= limit
    except (ValueError, TypeError):
        return False


class InputState:
    """Чистая логика разрешения движения, без сети и графического интерфейса."""

    def __init__(self):
        self.keys: set[str] = set()
        self.focused = False
        self.arm_intent = False
        self.arm_ack_deadline = -math.inf
        self.awaiting_arm_ack = False
        self.status: dict = {}
        self.status_time = -math.inf
        self.frame_time = -math.inf
        self.reason = "Ждём соединение и камеру"
        self.blocked_keys = set()

    def press(self, key):
        if key in self.blocked_keys:
            return
        if not self.arm_intent or self.status.get('phase') != 'ready':
            self.blocked_keys.add(key)
            return
        self.keys.add(key)

    def release(self, key):
        self.keys.discard(key)
        self.blocked_keys.discard(key)

    def update_status(self, message: dict, now: float):
        self.status = dict(message)
        self.status_time = now
        if message.get("phase") in ("arming", "ready") and self.arm_intent:
            self.awaiting_arm_ack = False
        if message.get("phase") in ("stopped", "error"):
            # Ответ, отправленный до получения arm, может прийти позже нажатия.
            # Он не разрешает движение: ждём arming/ready максимум 0,8 с.
            if message.get("phase") == "stopped" and self.awaiting_arm_ack and now < self.arm_ack_deadline:
                return
            self.arm_intent = False
            self.awaiting_arm_ack = False
            self.blocked_keys.update(self.keys)
            self.keys.clear()
            self.reason = str(message.get("reason") or "Управление остановлено")

    def healthy(self, now: float) -> tuple[bool, str]:
        if not self.focused:
            return False, "Окно не активно — движение запрещено"
        if not finite_age(now - self.status_time, 0.30):
            return False, "Нет свежего подтверждения от Raspberry Pi"
        if not isinstance(self.status.get("token"), int):
            return False, "Нет подтверждения связи"
        if not finite_age(now - self.frame_time, 0.80):
            return False, "Камера не обновляется"
        if not finite_age(self.status.get("telemetry_age_s"), 0.50):
            return False, "Нет свежих данных состояния робота"
        if not finite_age(self.status.get("camera_age_s"), 1.0):
            return False, "Камера робота не обновляется"
        if self.status.get("phase") == "error":
            return False, str(self.status.get("reason") or "Ошибка Raspberry Pi")
        return True, "Связь и камера доступны"

    def request_arm(self, now: float) -> dict | None:
        ok, reason = self.healthy(now)
        if not ok:
            self.reason = reason
            return None
        if self.keys or self.blocked_keys:
            self.reason = "Сначала отпустите клавиши движения"
            return None
        if self.arm_intent:
            self.reason = "Разрешение уже отправлено"
            return None
        self.arm_intent = True
        self.awaiting_arm_ack = True
        self.arm_ack_deadline = now + .8
        self.reason = "Подготовка режима робота; дождитесь готовности"
        return {"type": "arm", "token": self.status["token"]}

    def stop(self, reason: str) -> dict:
        self.blocked_keys.update(self.keys)
        self.keys.clear()
        self.arm_intent = False
        self.awaiting_arm_ack = False
        self.reason = reason
        return {"type": "stop", "reason": reason}

    def focus_lost(self) -> dict:
        self.focused = False
        return self.stop("Окно не активно — остановлено")

    def heartbeat(self, now: float) -> dict | None:
        ok, reason = self.healthy(now)
        if not ok:
            if self.arm_intent or self.keys:
                return self.stop(reason)
            self.reason = reason
            return None
        phase = self.status.get("phase")
        if self.awaiting_arm_ack and now >= self.arm_ack_deadline:
            return self.stop("Разрешение не подтверждено — управление остановлено")
        # Во время подготовки продолжаем подтверждать связь, но не движение.
        deadman = bool(self.arm_intent and phase == "ready" and self.status.get("armed") and self.keys & KEYS)
        if not self.arm_intent:
            self.reason = str(self.status.get("reason") or "Автоматическая подготовка; отпустите клавиши")
        elif phase == "ready":
            self.reason = "Клавиша удерживается — движение" if deadman else "Готово. Удерживайте клавишу направления"
        return {"type": "input", "token": self.status["token"], "deadman": deadman, "keys": sorted(self.keys & KEYS) if deadman else []}


class Outbox:
    """Ограниченная очередь: только последнее управление, остановка приоритетна.

    Запись в SSH выполняет отдельный поток; главный поток никогда не ждёт pipe.
    Номер seq присваивается непосредственно при выдаче, поэтому не меняет порядок.
    """

    def __init__(self, control_limit=8):
        self.condition = threading.Condition()
        self.controls = deque()
        self.pending_input = None
        self.closed = False
        self.sequence = 0
        self.control_limit = control_limit

    def put(self, message: dict, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        with self.condition:
            if self.closed:
                return False
            item = (dict(message), now)
            kind = message.get("type")
            if kind in ("stop", "quit"):
                # Закрытие терминально: поздний stop не должен поглотить quit.
                if any(old[0].get("type") == "quit" for old in self.controls):
                    return True
                self.controls.clear()
                self.pending_input = None
                self.controls.append(item)
            elif kind == "input":
                if any(old[0].get("type") == "quit" for old in self.controls):
                    return False
                self.pending_input = item
            else:
                if len(self.controls) >= self.control_limit or any(old[0].get("type") == "quit" for old in self.controls):
                    return False
                self.controls.append(item)
            self.condition.notify()
            return True

    def take(self, timeout=.1, now: float | None = None) -> dict | None:
        with self.condition:
            if not self.closed and not self.controls and self.pending_input is None:
                self.condition.wait(timeout)
            if self.controls:
                message, created = self.controls.popleft()
            elif self.pending_input is not None:
                message, created = self.pending_input
                self.pending_input = None
            else:
                return None
            current = time.monotonic() if now is None else now
            # Запоздалое движение не должно копиться в локальной очереди.
            if message.get("type") in ("input", "arm") and current - created > .15:
                message = {"type": "stop"}
            self.sequence += 1
            return {**message, "seq": self.sequence}

    def close(self):
        with self.condition:
            self.closed = True
            self.controls.clear()
            self.pending_input = None
            self.condition.notify_all()


class SSHTransport:
    def __init__(self, log_path: Path, command: list[str] | None = None):
        self.outbox = Outbox()
        self.lock = threading.Lock()
        self.latest = {}
        self.events = queue.Queue(maxsize=32)
        self.proc = None
        self.log_path = log_path
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log = self.log_path.open("a", encoding="utf-8", buffering=1)
        self.command = command or self.ssh_command()
        self.done = threading.Event()

    @staticmethod
    def ssh_command(host="192.168.11.81", key=None):
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.-]*", host):
            raise ValueError("Некорректный адрес Raspberry Pi")
        key = Path(key) if key else Path.home() / ".ssh/go2-pi-0008/id_ed25519"
        return [
            shutil.which("ssh") or str(Path(os.environ.get("WINDIR", "C:/Windows")) / "System32/OpenSSH/ssh.exe"), "-T", "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=10", "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=3",
            "-o", "IdentitiesOnly=yes", "-o", "StrictHostKeyChecking=yes",
            "-i", str(key), f"ubuntu@{host}",
            "/home/ubuntu/ai-robot/venv/bin/python /home/ubuntu/ai-robot/team_wolf_setup/teleop/run_teleop.py",
        ]

    def event(self, message):
        try:
            self.events.put_nowait(message)
        except queue.Full:
            # Достаточно последнего сообщения; поток чтения не блокируем.
            try:
                self.events.get_nowait()
            except queue.Empty:
                pass
            self.events.put_nowait(message)

    def start(self):
        threading.Thread(target=self._start, daemon=True, name="ssh-start").start()

    def _start(self):
        try:
            self.proc = subprocess.Popen(self.command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                         text=True, encoding="utf-8", errors="replace", bufsize=1,
                                         creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            threading.Thread(target=self._writer, daemon=True, name="ssh-write").start()
            threading.Thread(target=self._stderr, daemon=True, name="ssh-stderr").start()
            self._reader()
            result = self.proc.wait()
            self.event({"type": "transport_end", "exit_code": result})
        except Exception as exc:
            self.event({"type": "transport_error", "reason": f"Не удалось открыть SSH: {type(exc).__name__}"})
        finally:
            self.outbox.close()
            self.done.set()

    def _reader(self):
        while True:
            line = self.proc.stdout.readline(2_000_001)
            if not line:
                return
            if len(line) > 2_000_000:
                self.send({"type": "stop"})
                self.event({"type": "transport_error", "reason": "Слишком большое сообщение SSH"})
                continue
            try:
                message = json.loads(line)
                if not isinstance(message, dict):
                    raise ValueError("message")
            except (ValueError, TypeError):
                self.log.write("[stdout] " + line[:1000])
                continue
            kind = message.get("type")
            if kind in ("status", "frame"):
                with self.lock:
                    self.latest[kind] = (message, time.monotonic())
            else:
                self.event(message)
                if kind == "session_end":
                    self.log.write(json.dumps(message, ensure_ascii=False) + "\n")

    def _writer(self):
        try:
            while not self.outbox.closed:
                message = self.outbox.take()
                if message is None:
                    continue
                self.proc.stdin.write(json.dumps(message, ensure_ascii=True) + "\n")
                self.proc.stdin.flush()
                if message["type"] == "quit":
                    self.proc.stdin.close()
                    return
        except (BrokenPipeError, OSError, ValueError):
            self.event({"type": "transport_error", "reason": "Канал управления SSH закрыт"})

    def _stderr(self):
        for line in self.proc.stderr:
            self.log.write(line)

    def send(self, message):
        return self.outbox.put(message)

    def snapshot(self):
        with self.lock:
            result, self.latest = self.latest, {}
            return result


class DemoTransport:
    """Демонстрация окна: никакой сети, только синтетические кадры и статусы."""
    def __init__(self):
        self.events = queue.Queue()
        self.done = threading.Event()
        self.armed = False
        self.token = 0
        self.last_frame = -math.inf
        self.sent = []
        self.profile = 'precision'
        self.recording = None
        self.saved_recording = None

    def start(self):
        pass

    def send(self, message):
        self.sent.append(dict(message))
        self.sent = self.sent[-100:]
        kind = message["type"]
        if kind == "arm":
            self.armed = True
        elif kind in ("stop", "quit"):
            self.armed = False
        elif kind == 'profile':
            self.profile = message['profile']
            self.armed = False
        elif kind == 'record_start':
            self.recording = 'demo_run'
        elif kind == 'record_stop':
            self.saved_recording, self.recording = self.recording, None
        if kind == "quit":
            self.events.put({"type": "session_end", "session_dir": "Демонстрация: запись не создаётся", "fleet_restored": True, "exit_code": 0})
            self.done.set()
        return True

    def snapshot(self):
        from PIL import Image, ImageDraw
        now = time.monotonic()
        self.token += 1
        result = {"status": ({"type": "status", "phase": "ready" if self.armed else "waiting_arm", "token": self.token,
                              'protocol_version': 2, 'profile': self.profile, 'recording': self.recording,
                              'saved_recording': self.saved_recording,
                              "telemetry_age_s": .01, "camera_age_s": .03, "session_dir": "Демонстрация: запись не создаётся",
                              "armed": self.armed, "reason": "Робот не подключён"}, now)}
        if now - self.last_frame >= .45:
            self.last_frame = now
            picture = Image.new("RGB", (640, 360), "#142938")
            draw = ImageDraw.Draw(picture)
            draw.rectangle((50, 130, 590, 310), outline="#b6d7e8", width=3)
            draw.line((320, 360, 320, 160), fill="#78d9ad", width=4)
            draw.text((20, 25), "DEMO / NO ROBOT CONNECTED", fill="white")
            buffer = io.BytesIO()
            picture.save(buffer, "JPEG")
            result["frame"] = ({"type": "frame", "jpeg": base64.b64encode(buffer.getvalue()).decode(), "elapsed_s": 0, "seq": self.token}, now)
        return result


class KeyboardWindow:
    def __init__(self, tkroot, transport, dry_run=False):
        import tkinter as tk
        from tkinter import ttk
        self.root, self.transport, self.dry_run = tkroot, transport, dry_run
        self.state = InputState()
        self.photo = None
        self.closing_since = None
        self.session_end = None
        self.last_heartbeat = -math.inf
        self.release_after = {}
        self.last_auto_arm = -math.inf
        self.pending_action = None
        tkroot.title("Go2 — клавиатура и запись" + (" [ДЕМО БЕЗ РОБОТА]" if dry_run else ""))
        tkroot.geometry("950x850")
        tkroot.minsize(780, 730)
        tkroot.configure(bg="#eef3f5")
        frame = ttk.Frame(tkroot, padding=16)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="Go2 · тренировочное управление", font=("Segoe UI", 19, "bold")).pack(anchor="w")
        ttk.Label(frame, text="Удерживать W/S: вперёд/назад   •   A/D: поворот   •   Q/E: вбок", font=("Segoe UI", 11)).pack(anchor="w", pady=(6, 0))
        ttk.Label(frame, text="Без Пробела. Отпустить клавиши — остановка. X / Esc — стоп. Смена окна — стоп.", wraplength=880).pack(anchor="w", pady=(3, 8))
        if dry_run:
            tk.Label(frame, text="ДЕМОНСТРАЦИЯ: подключения к роботу нет", fg="#b02a1d", font=("Segoe UI", 12, "bold")).pack()
        camera_box = tk.Frame(frame, bg="#142938", height=360)
        camera_box.pack(fill="both", expand=True, pady=8)
        camera_box.pack_propagate(False)
        self.camera = tk.Label(camera_box, text="Ожидание камеры…\nПодготовка соединения может занять около минуты.", bg="#142938", fg="white")
        self.camera.pack(fill="both", expand=True)
        self.status_var = tk.StringVar(value="Ждём подключения…")
        self.details_var = tk.StringVar(value="Движение запрещено")
        self.path_var = tk.StringVar(value="Запись ещё не подтверждена")
        self.status_label = tk.Label(frame, textvariable=self.status_var, bg="#eef3f5", fg="#963022", font=("Segoe UI", 13, "bold"), anchor="w", wraplength=880)
        self.status_label.pack(fill="x", pady=(5, 0))
        ttk.Label(frame, textvariable=self.details_var, wraplength=880).pack(anchor="w", pady=4)
        ttk.Label(frame, text="Пределы API: вперёд/назад 0,6 м/с · вбок 0,5 м/с · поворот 1 рад/с").pack(anchor="w")
        row = ttk.Frame(frame)
        row.pack(fill="x", pady=10)
        self.profile_ids = ['precision', 'obstacle', 'floor', 'fast']
        self.profile_box = ttk.Combobox(row, state='readonly', width=30,
            values=['Точный: 0,10 / 0,08 / 0,20', 'Препятствие: 0,15 / 0,10 / 0,25',
                    'Пол: 0,30 / 0,15 / 0,50', 'Быстрый: 0,60 / 0,50 / 1,00'])
        self.profile_box.current(0)
        self.profile_box.pack(side='left')
        self.profile_box.bind('<<ComboboxSelected>>', self.change_profile)
        self.stop_button = tk.Button(row, text="СТОП  [X]", command=self.stop, bg="#b62924", fg="white", font=("Segoe UI", 15, "bold"), activebackground="#922018", activeforeground="white")
        self.stop_button.pack(side="left", fill="x", expand=True, padx=12)
        ttk.Button(row, text="Завершить и сохранить", command=self.close).pack(side="right", ipady=8)
        record_row = ttk.Frame(frame)
        record_row.pack(fill='x', pady=4)
        self.record_start_button = ttk.Button(record_row, text='● Начало записи', command=lambda: self.record('record_start'))
        self.record_start_button.pack(side='left', padx=4)
        self.record_stop_button = ttk.Button(record_row, text='■ Конец записи', command=lambda: self.record('record_stop'))
        self.record_stop_button.pack(side='left', padx=4)
        self.telemetry_var = tk.StringVar(value='Телеметрия: ожидание')
        ttk.Label(frame, textvariable=self.telemetry_var, wraplength=880).pack(anchor='w')
        marker_row = ttk.Frame(frame)
        marker_row.pack(fill="x", pady=3)
        self.marker_text = tk.StringVar(value="Начало препятствия")
        ttk.Label(marker_row, text="Отметка: ").pack(side="left")
        self.marker_entry = ttk.Entry(marker_row, textvariable=self.marker_text, width=35)
        self.marker_entry.pack(side="left", fill="x", expand=True)
        self.marker_entry.bind("<FocusIn>", lambda event: self.send(self.state.stop("Ввод подписи — движение запрещено")))
        ttk.Button(marker_row, text="Добавить в запись", command=self.marker).pack(side="left", padx=8)
        ttk.Label(frame, textvariable=self.path_var, wraplength=880, foreground="#426273").pack(anchor="w", pady=(8, 0))
        ttk.Label(frame, text="Скорости: вперёд / вбок (м/с) / поворот (рад/с). Запись по кнопке — на Pi; диагностический журнал — весь сеанс.", wraplength=880).pack(anchor="w", pady=(7, 0))
        # Перехватываем Пробел ДО стандартной привязки кнопок Tk:
        # иначе удержание Пробела может нажимать кнопку, имеющую фокус.
        key_tag = "Go2KeyboardFirst"
        tkroot.bind_class(key_tag, "<KeyPress>", self.key_press)
        tkroot.bind_class(key_tag, "<KeyRelease>", self.key_release)
        def install_key_tag(widget):
            widget.bindtags((key_tag,) + tuple(widget.bindtags()))
            for child in widget.winfo_children():
                install_key_tag(child)
        install_key_tag(tkroot)
        tkroot.bind("<FocusOut>", lambda event: tkroot.after_idle(self.check_focus))
        tkroot.bind("<FocusIn>", lambda event: self.check_focus())
        tkroot.protocol("WM_DELETE_WINDOW", self.close)
        transport.start()
        tkroot.after(25, self.tick)

    def send(self, message):
        if not self.transport.send(message) and message.get("type") not in ("stop", "quit"):
            self.state.stop("Канал управления перегружен — остановлено")
            self.transport.send({"type": "stop"})

    def check_focus(self):
        if self.closing_since is not None:
            return
        focused = self.root.focus_displayof() is not None
        if self.state.focused and not focused:
            self.send(self.state.focus_lost())
        else:
            self.state.focused = focused

    def typing(self):
        return self.root.focus_get() in (self.marker_entry, self.profile_box)

    def key_press(self, event):
        key = key_name(event.keysym, event.keycode)
        if key in self.release_after:
            self.root.after_cancel(self.release_after.pop(key))
        if key == "escape":
            self.stop()
            return "break"
        # В поле подписи можно печатать без движения. X в нём — обычная буква.
        if self.typing():
            if key in KEYS:
                self.state.blocked_keys.add(key)
            if self.state.arm_intent or self.state.keys:
                self.send(self.state.stop("Ввод подписи — движение запрещено"))
            return None
        if key == "x":
            self.stop()
            return "break"
        if self.closing_since is None and key in KEYS:
            self.state.press(key)
            return "break"
        if key == 'space':
            return 'break'
        return None

    def key_release(self, event):
        key = key_name(event.keysym, event.keycode)
        if key in KEYS | {"space"}:
            # На Windows autorepeat не отпускает клавишу. Малое after_idle также
            # нейтрализует пары Release/Press на других платформах.
            def release():
                self.release_after.pop(key, None)
                self.state.release(key)
                if self.closing_since is None:
                    message = self.state.heartbeat(time.monotonic())
                    if message:
                        self.send(message)
            self.release_after[key] = self.root.after_idle(release)
            return "break"

    def arm(self):
        self.check_focus()
        if self.closing_since is None:
            message = self.state.request_arm(time.monotonic())
            if message:
                self.send(message)

    def stop(self):
        self.last_auto_arm = time.monotonic()
        self.send(self.state.stop("Остановлено. Отпустите клавиши, затем нажмите направление заново"))

    def change_profile(self, event=None):
        self.stop()
        self.pending_action = {'type': 'profile', 'profile': self.profile_ids[self.profile_box.current()]}
        self.root.focus_set()

    def record(self, action):
        self.stop()
        self.pending_action = {'type': action}
        self.root.focus_set()

    def marker(self):
        self.stop()
        label = self.marker_text.get().strip()[:200]
        if label and self.closing_since is None:
            self.send({"type": "marker", "label": label})
            self.state.reason = f"Отметка отправлена: {label}"

    def close(self):
        if self.closing_since is not None:
            return
        self.state.stop("Завершаем запись и восстанавливаем штатное подключение…")
        self.send({"type": "quit"})
        self.closing_since = time.monotonic()
        self.profile_box.configure(state="disabled")

    def tick(self):
        from PIL import Image, ImageTk
        now = time.monotonic()
        self.check_focus()
        snapshot = self.transport.snapshot()
        if "status" in snapshot:
            message, received = snapshot["status"]
            self.state.update_status(message, received)
            run = message.get('recording')
            saved = message.get('saved_recording')
            self.path_var.set(('● ПИШЕМ: ' + run if run else 'Запись выключена; сохранено: ' + str(saved or 'ещё нет'))
                              + '\nPi: ' + str(message.get('session_dir', 'ожидание')))
            tele = message.get('telemetry') or {}
            self.telemetry_var.set('Профиль Pi: ' + str(message.get('profile', '—')) +
                ' · возраст состояния: ' + str(message.get('telemetry_age_s', '—'))[:5] + ' с' +
                ' · скорость по датчикам: ' + str(tele.get('velocity', '—')) +
                ' · наклон r/p/y (рад): ' + str((tele.get('imu_state') or {}).get('rpy', '—')))
        if "frame" in snapshot:
            message, received = snapshot["frame"]
            try:
                raw = base64.b64decode(message["jpeg"], validate=True)
                picture = Image.open(io.BytesIO(raw))
                if picture.width > 1920 or picture.height > 1080:
                    raise ValueError("frame size")
                picture.load()
                picture.thumbnail((850, 430))
                self.photo = ImageTk.PhotoImage(picture)
                self.camera.configure(image=self.photo, text="", width=640, height=360)
                self.state.frame_time = received
            except Exception:
                self.send(self.state.stop("Повреждённый кадр камеры — остановлено"))
        while True:
            try:
                event = self.transport.events.get_nowait()
            except queue.Empty:
                break
            if event.get("type") in ("transport_error", "transport_end"):
                reason = str(event.get("reason") or f"Соединение закрыто, код {event.get('exit_code')}")
                self.state.stop(reason)
                self.state.status.update({"phase": "error", "reason": reason})
            elif event.get("type") == "progress":
                self.path_var.set(str(event.get("reason", "Подключение…")))
            elif event.get("type") == "session_end":
                self.session_end = event
                self.state.stop("Запись завершена")
                self.state.status.update({"phase": "error", "reason": "Сеанс записи завершён"})
                self.path_var.set("Сохранено на Pi: " + str(event.get("session_dir", "неизвестно")))
        if self.closing_since is None and now - self.last_heartbeat >= .095:
            self.last_heartbeat = now
            healthy, _ = self.state.healthy(now)
            compatible = self.state.status.get('protocol_version') == 2
            can_finish = bool(self.pending_action and self.pending_action.get('type') == 'record_stop'
                              and finite_age(now - self.state.status_time, .30))
            if (healthy or can_finish) and compatible and self.pending_action:
                self.send({**self.pending_action, 'token': self.state.status['token']})
                self.pending_action = None
                self.last_auto_arm = now
            elif healthy and compatible and not self.typing() and not self.state.arm_intent and now - self.last_auto_arm > 1.5:
                self.last_auto_arm = now
                self.arm()
            message = self.state.heartbeat(now)
            if message:
                self.send(message)
        if self.closing_since is not None:
            if self.transport.done.is_set():
                if self.session_end and self.session_end.get("fleet_restored"):
                    self.root.destroy()
                    return
                self.status_var.set("Соединение закрыто. Восстановление службы не подтверждено — сообщите оператору")
                self.details_var.set("Окно можно закрыть повторным нажатием крестика. Проверьте журнал подключения.")
                self.root.protocol("WM_DELETE_WINDOW", self.root.destroy)
            else:
                elapsed = int(now - self.closing_since)
                self.status_var.set(f"Остановка и сохранение · ожидание завершения {elapsed} с")
                self.details_var.set("Ждём восстановления штатной службы. Команды движения запрещены." if elapsed <= 35 else "Восстановление занимает больше 35 с. Ожидаем подтверждения; окно пока не закрываем.")
        else:
            self.status_var.set(self.state.reason)
            phase = self.state.status.get("phase", "соединение")
            self.details_var.set(f"Состояние: {phase} · клавиши: {' + '.join(sorted(self.state.keys)).upper() or 'отпущены'}")
        healthy, _ = self.state.healthy(now)
        self.status_label.configure(fg="#116648" if self.state.arm_intent and healthy else "#963022")
        if self.closing_since is None:
            compatible = self.state.status.get('protocol_version') == 2
            if self.state.status and not compatible:
                self.status_var.set('Нужна версия 2 на Pi: обновите оба серверных файла по инструкции')
            self.record_start_button.configure(state='normal' if healthy and compatible and not self.state.status.get('recording') and not self.pending_action else 'disabled')
            self.record_stop_button.configure(state='normal' if finite_age(now - self.state.status_time, .30) and compatible and self.state.status.get('recording') and not self.pending_action else 'disabled')
        self.root.after(25, self.tick)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", help="Показать демонстрацию без подключения")
    parser.add_argument("--host", default="192.168.11.81", help="Адрес назначенного Raspberry Pi")
    parser.add_argument("--key", type=Path, default=Path.home() / ".ssh/go2-pi-0008/id_ed25519", help="Путь к SSH-ключу")
    parser.add_argument("--log-dir", type=Path, default=WORKSPACE / "outputs/go2-teleop/client-logs")
    args = parser.parse_args()
    import tkinter as tk
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    transport = DemoTransport() if args.dry_run else SSHTransport(args.log_dir / f"keyboard-{timestamp}.log", SSHTransport.ssh_command(args.host, args.key))
    root = tk.Tk()
    KeyboardWindow(root, transport, args.dry_run)
    root.mainloop()


if __name__ == "__main__":
    main()
