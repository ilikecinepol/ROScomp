"""Замкнутая кинематическая проверка, без физики и подключения к роботу.

Размеченные геометрия, опора и локализация здесь идеальны и синтетические.
Поза меняется только интегрированием выданной команды. Мир не знает фаз,
цели и внутренних опорных точек MissionController. Эти искусственные навыки
не сохраняются как профиль и не разрешают движение реального Go2.
"""
from __future__ import annotations

from collections import deque
from dataclasses import replace
import math
import random

from .mission import MissionController
from .models import Decision, Grid, KINDS, Observation, Obstacle, Policy, Pose, StartLine

SCOPE = 'kinematic_not_physics_not_robot_validation'


def integrate_pose(pose: Pose, decision: Decision, dt: float) -> Pose:
    """Точное интегрирование постоянной скорости корпуса на одном интервале."""
    values = (pose.x, pose.y, pose.yaw, pose.z, decision.vx, decision.vy, decision.wz, dt)
    if not all(math.isfinite(value) for value in values) or dt <= 0:
        raise ValueError('Кинематике нужны конечные значения и положительный dt')
    theta = pose.yaw
    turn = decision.wz * dt
    if abs(decision.wz) < 1e-10:
        dx = (decision.vx * math.cos(theta) - decision.vy * math.sin(theta)) * dt
        dy = (decision.vx * math.sin(theta) + decision.vy * math.cos(theta)) * dt
    else:
        last = theta + turn
        dx = (decision.vx * (math.sin(last) - math.sin(theta))
              + decision.vy * (math.cos(last) - math.cos(theta))) / decision.wz
        dy = (decision.vx * (math.cos(theta) - math.cos(last))
              + decision.vy * (math.sin(last) - math.sin(theta))) / decision.wz
    return Pose(pose.x + dx, pose.y + dy,
                math.atan2(math.sin(theta + turn), math.cos(theta + turn)), pose.z)


class SyntheticWorld:
    """Неподвижная сцена; данные доступны как идеальные наблюдения, не в FSM.

    Пустой пол целиком измерен виртуальным датчиком. Поверхности — гладкие
    кинематические профили, включая условный балансир без качания. Ступени,
    контакт лап, скольжение, опрокидывание и окклюзии не моделируются.
    """
    def __init__(self, seed=0):
        rng = random.Random(seed)
        self.rotation = rng.uniform(-math.pi, math.pi)
        self.translation = (rng.uniform(-4, 4), rng.uniform(-4, 4))
        self._cos, self._sin = math.cos(self.rotation), math.sin(self.rotation)
        initial = self._point((0., rng.uniform(-.25, .25), 0.))
        self.initial_pose = Pose(initial[0], initial[1], self.rotation + rng.uniform(-.3, .3), .32)
        self.line_center = self._point((.25, 0., 0.))[:2]
        self.line_normal = (self._cos, self._sin)
        self.line_endpoints = (self._point((.25, -1.5, 0.))[:2],
                               self._point((.25, 1.5, 0.))[:2])
        profiles = {
            'aframe': ((0., 0., 0.), (.45, 0., .12), (.9, 0., 0.)),
            'bridge': ((0., 0., 0.), (.35, 0., .04), (.7, 0., .04), (1.05, 0., 0.)),
            'teeter': ((0., 0., 0.), (.5, 0., .08), (1., 0., 0.)),
            'slalom': ((0., 0., 0.), (.35, .16, 0.), (.7, -.16, 0.), (1.05, .16, 0.), (1.4, 0., 0.)),
            'platforms': ((0., 0., 0.), (.3, 0., .08), (.6, 0., 0.), (.9, 0., .08), (1.2, 0., 0.)),
        }
        placements = [(1.2, -1.5, 0.), (3.3, -1.5, 0.),
                      (4.7, .4, math.pi / 2), (3.5, 2., math.pi), (1.3, 2., math.pi)]
        kinds = list(KINDS)
        rng.shuffle(kinds)
        self.obstacles = []
        for index, (kind, (x0, y0, heading)) in enumerate(zip(kinds, placements)):
            c, s = math.cos(heading), math.sin(heading)
            path = tuple(self._point((x0 + x*c - y*s, y0 + x*s + y*c, z))
                         for x, y, z in profiles[kind])
            self.obstacles.append(Obstacle(
                id='synthetic-%s-%s' % (seed, index), kind=kind, path=path,
                width=1.3, observed_at=0., confidence=1., confirmations=4,
                geometry_verified=True, direction_verified=True, bidirectional=False,
                max_slope_deg=20., max_step_m=.08 if kind == 'platforms' else 0.,
                evidence=('идеальная синтетическая разметка; не реальный датчик',)))
        points = [point for item in self.obstacles for point in item.path] + [initial]
        margin, resolution = 2., .4
        minimum = (min(p[0] for p in points) - margin, min(p[1] for p in points) - margin)
        maximum = (max(p[0] for p in points) + margin, max(p[1] for p in points) + margin)
        width = math.ceil((maximum[0] - minimum[0]) / resolution)
        height = math.ceil((maximum[1] - minimum[1]) / resolution)
        self.grid = Grid(minimum, resolution, [[0] * width for _ in range(height)], verified=True)

    def _point(self, point):
        x, y, z = point
        return (self.translation[0] + self._cos*x - self._sin*y,
                self.translation[1] + self._sin*x + self._cos*y, z)

    def support_height(self, pose):
        """Интерполяция ближайшего сегмента сцены, независимо от цели FSM."""
        candidates = []
        for item in self.obstacles:
            for a, b in zip(item.path, item.path[1:]):
                dx, dy = b[0] - a[0], b[1] - a[1]
                norm2 = dx*dx + dy*dy
                fraction = min(1., max(0., ((pose.x-a[0])*dx + (pose.y-a[1])*dy) / norm2))
                distance = math.hypot(pose.x-a[0]-fraction*dx, pose.y-a[1]-fraction*dy)
                if distance <= item.width / 2:
                    candidates.append((distance, a[2] + fraction * (b[2] - a[2])))
        return min(candidates)[1] if candidates else 0.

    def signed_line_distance(self, pose):
        return ((pose.x-self.line_center[0])*self.line_normal[0]
                + (pose.y-self.line_center[1])*self.line_normal[1])

    def line_crossed(self, previous, current):
        """Импульс пересечения конечного отрезка в любую сторону."""
        a, b = self.signed_line_distance(previous), self.signed_line_distance(current)
        if not (a < 0 <= b or b < 0 <= a):
            return False
        fraction = a / (a-b)
        x = previous.x + fraction*(current.x-previous.x) - self.line_center[0]
        y = previous.y + fraction*(current.y-previous.y) - self.line_center[1]
        lateral = -x*self.line_normal[1] + y*self.line_normal[0]
        return abs(lateral) <= 1.5

    def observe(self, t, pose, previous, speed):
        support = self.support_height(pose)
        measured = replace(pose, z=support + .32)
        visible = abs(self.signed_line_distance(pose)) < 1.2
        return Observation(t, measured, self.grid,
            [replace(item, observed_at=t) for item in self.obstacles],
            localized=True, localization_error_m=.01, start_line_visible=visible,
            start_line_crossed=self.line_crossed(previous, pose), body_height=.32,
            speed=speed, support_height_m=support, support_verified=True,
            diagnostics=['идеальная синтетическая геометрия и локализация'],
            start_line=StartLine(self.line_endpoints, t, confidence=1., frame_epoch=0,
                                 geometry_verified=True, confirmations=4) if visible else None)


def run_simulation(seed=0, max_steps=4500, *, latency_steps=None, drop_every=0):
    """JSON-совместимый результат настоящего цикла команда→поза→наблюдение.

    Задержка — очередь целых снимков; пропуск пакета удерживает предыдущую
    команду на один dt. Это испытание MissionController, а не watchdog runtime.
    Никакой профиль для реального робота эта функция не создаёт.
    """
    if type(seed) is not int or type(max_steps) is not int or max_steps < 1:
        raise ValueError('seed должен быть целым; max_steps — положительным целым')
    latency = abs(seed) % 3 if latency_steps is None else latency_steps
    if type(latency) is not int or latency < 0 or latency > 4:
        raise ValueError('latency_steps должен быть целым от 0 до 4')
    if type(drop_every) is not int or drop_every < 0:
        raise ValueError('drop_every должен быть неотрицательным целым')
    world = SyntheticWorld(seed)
    # Только локальная синтетика: не RobotProfile, не файл конфигурации live.
    policy = Policy(validated_skills=KINDS, required_kinds=KINDS, return_mode='line', max_mission_seconds=900.)
    controller = MissionController(policy)
    dt = .15
    pose = previous = world.initial_pose
    command = Decision()
    pending = deque()
    trajectory, transitions, crossings = [], [], []
    prior_phase = None
    delivered, dropped = 0, 0
    started = False
    t = 0.
    for step in range(max_steps):
        t = step * dt
        observed = world.observe(t, pose, previous, math.hypot(command.vx, command.vy))
        if observed.start_line_crossed:
            crossings.append({'t': t, 'x': pose.x, 'y': pose.y})
        pending.append(observed)
        if len(pending) > latency:
            snapshot = pending.popleft()
            if drop_every and step and step % drop_every == 0:
                dropped += 1
            else:
                command = controller.step(snapshot, start=not started)
                started = True
                delivered += 1
        if command.phase != prior_phase:
            transitions.append({'t': t, 'phase': command.phase, 'reason': command.reason,
                                'target_id': command.target_id})
            prior_phase = command.phase
        trajectory.append({'t': t, 'x': pose.x, 'y': pose.y, 'yaw': pose.yaw,
                           'z': observed.pose.z, 'vx': command.vx, 'vy': command.vy,
                           'wz': command.wz, 'phase': command.phase})
        if command.phase in ('FINISHED', 'FAULT'):
            break
        previous = pose
        pose = integrate_pose(pose, command, dt)
    return {
        'scope': SCOPE, 'seed': seed, 'completed': command.phase == 'FINISHED',
        'elapsed': t, 'steps': len(trajectory), 'dt': dt, 'final_phase': command.phase,
        'reason': command.reason, 'completed_kinds': list(command.completed),
        'trajectory': trajectory, 'transitions': transitions, 'line_crossings': crossings,
        'latency_steps': latency, 'latency_seconds': latency*dt,
        'delivered_observations': delivered, 'dropped_observations': dropped,
        'scene': {'start': {'x': world.initial_pose.x, 'y': world.initial_pose.y,
                            'yaw': world.initial_pose.yaw},
                  'rotation': world.rotation, 'translation': list(world.translation),
                  'line_center': list(world.line_center), 'line_normal': list(world.line_normal),
                  'line_endpoints': [list(point) for point in world.line_endpoints],
                  'obstacles': [{'id': item.id, 'kind': item.kind, 'path': [list(p) for p in item.path]}
                                for item in world.obstacles]},
        'limitations': ['Идеальные метки и видимость всех пяти конструкций.',
                        'Опора задаётся неподвижными гладкими геометрическими профилями.',
                        'Нет физики лап, ступеней, качания, скольжения, контактов или падений.',
                        'Нет проверки распознавания, реальной калибровки, API и аппаратного watchdog.'],
    }
