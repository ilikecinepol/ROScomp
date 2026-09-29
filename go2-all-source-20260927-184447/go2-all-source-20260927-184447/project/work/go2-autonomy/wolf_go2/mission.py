"""Миссия по наблюдениям: пять типов препятствий и подтверждённый возврат.

Модуль не подключается к роботу. Поля verified и validated_skills являются
контрактом адаптера и профиля; этот автомат не превращает их в аппаратные
доказательства. Точки поверхности сохраняют порядок, включая промежуточные
точки слалома и обеих платформ. Секундомер сам по себе ничего не засчитывает.
"""
from __future__ import annotations

from dataclasses import replace
import itertools
import math

from .models import Decision, Grid, KINDS, Observation, Obstacle, Policy, Pose, StartLine
from .navigation import astar, follow_path, frontier_path, optimize_route, path_is_clear


def _finite(*values):
    return all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v) for v in values)


def _xy(value):
    return (value.x, value.y) if isinstance(value, Pose) else tuple(value[:2])


def _angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _heading(path, end=False):
    pairs = list(zip(path, path[1:]))
    for a, b in reversed(pairs) if end else pairs:
        if math.dist(a[:2], b[:2]) > 1e-8:
            return math.atan2(b[1] - a[1], b[0] - a[0])
    return None


class MissionController:
    """Детерминированный step; кадр и разрешение старта предоставляет адаптер.

    При смене эпохи координат FAULT защёлкивается: повторный start не сбрасывает
    накопленные преобразования. Новый запуск требует нового экземпляра.
    Абсолютную свежесть относительно часов компьютера проверяет runtime;
    здесь проверяются порядок t, разрыв кадров и возраст кандидатов.
    """

    def __init__(self, policy: Policy, trial_kind=None, approach_only=False, commissioning=False):
        if commissioning and trial_kind is None:
            raise ValueError('Первичное испытание допускает только одно выбранное препятствие')
        self.commissioning=bool(commissioning)
        if trial_kind not in (None, 'aframe', 'platforms', 'slalom', 'teeter'):
            raise ValueError('Неизвестный вид отдельного испытания')
        if approach_only and trial_kind is None:
            raise ValueError('Для выравнивания нужен конкретный вид')
        self.trial_kind = trial_kind
        self.approach_only = approach_only
        self.policy = policy.validate()
        self._home_stable_since=None
        self._alignment_stable_since=None
        self.phase = "WAIT_START"
        self.completed_ids = set()
        self.completed_kinds = set()
        self.completion_order = []
        self.memory = {}
        self.route_order = []
        self.current = None
        self.waypoint_index = 0
        self.visited_waypoints = []
        self.start_pose = None
        self.frame_epoch = None
        self._last_t = None
        self._last_pose = None
        self._dt = 0.
        self._started_at = None
        self._phase_started = None
        self._progress_t = None
        self._route_wait_since = None
        self._route_wait_total = 0.
        self._best_distance = math.inf
        self._nav_path = []
        self._nav_index = 0
        self._approach_goal = None
        self._reverse = False
        self._entry_pose = None
        self._exit_goal = None
        self._verify_t = None
        self._verify_count = 0
        self._return_crossing_armed = False
        self._return_previous_crossed = False
        self._start_line = None
        self._return_stage = "approach"
        self._return_goal = None
        self._return_selected_along = None
        self._return_geometry_problem = None
        self._return_line_problem = None
        self._fault_reason = ""
        self._last_command = (0., 0.)

    def _decision(self, reason, vx=0., wz=0.):
        self._last_command = (float(vx), float(wz))
        return Decision(vx=float(vx), wz=float(wz), phase=self.phase, reason=reason,
            target_id=self.current.id if self.current else None, completed=tuple(self.completion_order))

    def _fault(self, reason):
        self.phase = "FAULT"
        self._fault_reason = reason
        return self._decision(reason)

    def _phase(self, name, now):
        self.phase = name
        self._alignment_stable_since=None
        self._phase_started = now
        self._progress_t = now
        self._route_wait_since = None
        self._route_wait_total = 0.
        self._best_distance = math.inf
        self._last_command = (0., 0.)

    def _wait(self, obs, reason):
        self._home_stable_since=None
        self._alignment_stable_since=None
        if self.phase == "VERIFY":
            self._verify_t = None
            self._verify_count = 0
        if self._progress_t is not None and obs.t - self._progress_t > self.policy.progress_timeout:
            return self._fault("Нет подтверждённого продвижения: " + reason)
        return self._decision(reason)

    def _mark_progress(self, distance, now):
        threshold = max(.005, self.policy.waypoint_tolerance * .15)
        if distance < self._best_distance - threshold:
            self._best_distance = distance
            self._progress_t = now

    def _wait_for_route(self, obs, label):
        """Ожидание занятого прохода ограничено суммарно на этап навигации."""
        if self._route_wait_since is None:
            self._route_wait_since = obs.t
        elapsed = self._route_wait_total + obs.t - self._route_wait_since
        if elapsed >= self.policy.route_wait_timeout:
            return self._fault("Истекло время ожидания свободного пути: " + label)
        return self._decision("Ожидаем подтверждённый свободный путь: " + label)

    def _resume_route(self, now):
        if self._route_wait_since is not None:
            paused = max(0., now - self._route_wait_since)
            self._route_wait_total += paused
            if self._progress_t is not None:
                self._progress_t += paused
            self._route_wait_since = None

    def _pose_errors(self, obs):
        if not obs.localized or obs.pose is None:
            return ["Нет подтверждённой локализации"]
        if not _finite(obs.pose.x, obs.pose.y, obs.pose.z, obs.pose.yaw, obs.localization_error_m):
            return ["Некорректная поза или её погрешность"]
        if not 0 <= obs.localization_error_m <= self.policy.max_localization_error:
            return ["Погрешность локализации выше профиля"]
        if obs.grid is None or not obs.grid.verified:
            return ["Нет достоверной карты свободного пространства"]
        if obs.grid.frame_epoch != obs.frame_epoch:
            return ["Эпохи позы и карты различаются"]
        if not path_is_clear(self._motion_grid(obs), [_xy(obs.pose)], self.policy, obs.localization_error_m):
            return ["Габарит робота и зона поворота не подтверждены свободными"]
        return []

    def _surface_grid(self, obs, item):
        if not isinstance(obs.traversal_grids,dict):
            return None
        if item.id not in obs.traversal_grids:
            return obs.grid
        grid = obs.traversal_grids[item.id]
        floor = obs.grid
        if (not isinstance(grid,Grid) or not isinstance(floor,Grid) or grid.verified is not True
                or grid.frame_epoch != obs.frame_epoch or grid.origin != floor.origin
                or grid.resolution != floor.resolution or len(grid.cells) != len(floor.cells)):
            return None
        # Карта поверхности не может тайно сделать неизвестное свободным
        # или открыть чужое препятствие вне измеренного коридора этого id.
        cleared_raised = False
        for y,(row,old_row) in enumerate(zip(grid.cells,floor.cells)):
            if len(row) != len(old_row):
                return None
            for x,(value,old) in enumerate(zip(row,old_row)):
                if value != 0 or old == 0:
                    continue
                if old != 1:
                    return None
                cleared_raised = True
                point = (grid.origin[0]+(x+.5)*grid.resolution,
                         grid.origin[1]+(y+.5)*grid.resolution)
                nearest = math.inf
                for a,b in zip(item.path,item.path[1:]):
                    dx,dy = b[0]-a[0],b[1]-a[1]
                    norm = dx*dx+dy*dy
                    fraction = min(1.,max(0.,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/norm)) if norm > 1e-12 else 0.
                    nearest = min(nearest,math.hypot(point[0]-a[0]-fraction*dx,point[1]-a[1]-fraction*dy))
                if nearest > item.width/2+grid.resolution/math.sqrt(2):
                    return None
        # Низкий пол рядом с боковой кромкой также имеет значение 0, однако
        # он не поддерживает поворот корпуса на высоте поверхности.
        # Поэтому нужен отдельный запас по подтверждённой ширине опоры.
        turn_width = math.hypot(self.policy.robot_length,self.policy.robot_width)+2*(self.policy.clearance+obs.localization_error_m)
        if cleared_raised and item.width < turn_width:
            return None
        return grid

    def _motion_grid(self, obs):
        if self.current is not None and self.phase in ("ALIGN","TRAVERSE","VERIFY"):
            return self._surface_grid(obs,self.current)
        return obs.grid

    def _staging_points(self, obs, item, path):
        offset = self.policy.robot_length/2+self.policy.clearance+obs.localization_error_m
        surface_map = isinstance(obs.traversal_grids,dict) and item.id in obs.traversal_grids
        if surface_map:
            offset = (math.hypot(self.policy.robot_length,self.policy.robot_width)/2
                      +self.policy.clearance+obs.localization_error_m+math.sqrt(2)*obs.grid.resolution)
        entry_heading,exit_heading = _heading(path),_heading(path,end=True)
        pre = (path[0][0]-offset*math.cos(entry_heading),path[0][1]-offset*math.sin(entry_heading))
        post = (path[-1][0]+offset*math.cos(exit_heading),path[-1][1]+offset*math.sin(exit_heading)) if surface_map else path[-1][:2]
        return pre,post

    def _candidate_errors(self, item, obs, check_grid=True):
        p = self.policy
        errors = []
        if not isinstance(item, Obstacle) or not isinstance(item.id, str) or not item.id or item.kind not in KINDS:
            return ["Некорректная идентичность препятствия"]
        if item.frame_epoch != obs.frame_epoch:
            errors.append("Препятствие в другой эпохе координат")
        if not _finite(item.observed_at) or not 0 <= obs.t - item.observed_at <= p.max_candidate_age:
            errors.append("Устаревшее или будущее наблюдение препятствия")
        if item.geometry_verified is not True or item.direction_verified is not True:
            errors.append("Геометрия или направление не подтверждены")
        if (not _finite(item.confidence) or not .8 <= item.confidence <= 1.
                or type(item.confirmations) is not int or item.confirmations < 3):
            errors.append("Недостаточно подтверждений объекта")
        if item.kind not in p.validated_skills and not (self.commissioning and item.kind==self.trial_kind):
            errors.append("Навык не подтверждён профилем назначенного робота")
        if not _finite(item.width) or item.width < p.robot_width + 2 * (p.clearance + obs.localization_error_m):
            errors.append("Ширины не хватает с учётом зазора и погрешности")
        limits = ((item.max_step_m, p.max_step, "ступень"), (item.max_slope_deg, p.max_slope_deg, "уклон"),
            (item.max_gap_m, p.max_gap, "щель"), (item.max_lip_m, p.max_lip, "стык"))
        for value, limit, label in limits:
            if not _finite(value) or value < 0 or value > limit:
                errors.append("Превышен или неизвестен предел: " + label)
        if (not isinstance(item.path, (list, tuple)) or len(item.path) < 2
                or any(not isinstance(point, (list, tuple)) or len(point) != 3
                       or not _finite(*point) for point in item.path)):
            return errors + ["Нужен конечный наблюдённый путь 3D"]
        if math.dist(item.path[0][:2], item.path[-1][:2]) <= 2 * p.waypoint_tolerance:
            errors.append("Вход и выход не разделены достаточно для подтверждения прохождения")
        for a, b in zip(item.path, item.path[1:]):
            horizontal = math.dist(a[:2], b[:2])
            vertical = abs(a[2] - b[2])
            if item.kind == "platforms":
                if vertical > p.max_step + 1e-8:
                    errors.append("Наблюдённый перепад платформ выше профиля")
            elif math.degrees(math.atan2(vertical, horizontal)) > p.max_slope_deg + 1e-8:
                errors.append("Наблюдённый уклон пути выше профиля")
        if check_grid and not path_is_clear(self._surface_grid(obs,item), [point[:2] for point in item.path], p, obs.localization_error_m):
            errors.append("Путь поверхности проходит через занятые или неизвестные ячейки")
        return errors

    def _remember(self, obs):
        for item in obs.obstacles:
            if not isinstance(item, Obstacle) or not isinstance(item.id, str) or not item.id or item.kind not in KINDS or item.frame_epoch != obs.frame_epoch:
                continue
            old = self.memory.get(item.id)
            if old is not None and old.kind != item.kind:
                # Не переименовываем уже выбранный/пройденный объект молча.
                if (self.current and item.id == self.current.id) or item.id in self.completed_ids:
                    return "Изменилась классификация сохранённого объекта"
                continue
            if _finite(item.observed_at) and item.observed_at <= obs.t and (old is None or item.observed_at >= old.observed_at):
                self.memory[item.id] = item
        return None

    def _select(self, obs):
        available = [v for v in self.memory.values() if v.id not in self.completed_ids and v.kind not in self.completed_kinds
            and (self.trial_kind is None or v.kind == self.trial_kind)
            and (self.trial_kind is not None or v.kind in self.policy.required_kinds)
            and not self._candidate_errors(v, obs)]
        if self.trial_kind and len(available) != 1:
            return False
        # Для каждого типа выбираем ближайшее ДОСТИЖИМОЕ воплощение.
        # Дальше <=5 типов оптимизируются вместе с разрешёнными направлениями.
        by_kind = {}
        stages = {}
        surface_grids = {}
        for item in available:
            distance = math.inf
            surface_grid = self._surface_grid(obs,item)
            surface_grids[item.id] = surface_grid
            for reverse in ((False,True) if item.bidirectional else (False,)):
                path = tuple(reversed(item.path)) if reverse else item.path
                approach,post = self._staging_points(obs,item,path)
                route = astar(obs.grid, _xy(obs.pose), approach, self.policy, obs.localization_error_m)
                surface_route = [approach]+[p[:2] for p in path]+[post]
                if (route and path_is_clear(surface_grid,surface_route,self.policy,obs.localization_error_m)
                        and path_is_clear(obs.grid,[post],self.policy,obs.localization_error_m)):
                    stages[(item.id,reverse)] = (approach,post)
                    route = [_xy(obs.pose)] + route + [approach]
                    distance = min(distance, sum(math.dist(a,b) for a,b in zip(route,route[1:])))
            if not math.isfinite(distance):
                continue
            if item.kind not in by_kind or distance < by_kind[item.kind][0]:
                by_kind[item.kind] = (distance, item)
        items = [pair[1] for pair in by_kind.values()]
        order = []
        # Недостижимый дальний объект не отменяет подход к доступному подмножеству.
        for size in range(len(items), 0, -1):
            options = []
            for subset in itertools.combinations(items, size):
                route = optimize_route(obs.pose, list(subset), obs.grid, self.policy,
                    obs.localization_error_m, home=self.start_pose,
                    surface_grids=surface_grids,stages=stages)
                if route:
                    first = self.memory[route[0][0]]
                    entry = first.path[-1 if route[0][1] else 0][:2]
                    options.append((math.dist(_xy(obs.pose), entry), route))
            if options:
                order = min(options, key=lambda pair: pair[0])[1]
                break
        self.route_order = list(order)
        for item_id, reverse in order:
            item = self.memory[item_id]
            path = tuple(reversed(item.path)) if reverse else item.path
            stage = stages.get((item_id,reverse))
            if stage is None:
                continue
            approach,post = stage
            route = astar(obs.grid, _xy(obs.pose), approach, self.policy, obs.localization_error_m)
            if not route:
                continue
            self.current = replace(item, path=path)
            self._reverse = reverse
            self._approach_goal = approach
            self._nav_path = route + [approach]
            self._nav_index = 0
            self.waypoint_index = 0
            self.visited_waypoints = []
            self._entry_pose = None
            self._exit_goal = post
            self._phase("APPROACH", obs.t)
            return True
        return False

    def _refresh_target(self, obs):
        item = self.memory.get(self.current.id)
        if item is None:
            return ["Нет наблюдения выбранного объекта"]
        if self._reverse and item.bidirectional is not True:
            return ["Отозвано подтверждение обратного направления"]
        errors = self._candidate_errors(item, obs)
        if errors:
            return errors
        path = tuple(reversed(item.path)) if self._reverse else item.path
        old = self.current.path
        if len(path) != len(old) or any(math.dist(a, b) > self.policy.waypoint_tolerance for a, b in zip(path, old)):
            return ["Геометрия выбранного пути существенно изменилась"]
        # Порядок опорных точек остаётся закреплённым на всё прохождение.
        self.current = replace(item, path=old)
        return []

    def _drive(self, obs, target, speed, reason, stop_at_end=True):
        segment = [_xy(obs.pose), tuple(target[:2])]
        if not path_is_clear(self._motion_grid(obs), segment, self.policy, obs.localization_error_m):
            return self._wait(obs, "Ближайший участок не подтверждён свободным")
        vx, wz = follow_path(obs.pose, segment, self.policy, speed_limit=speed, stop_at_end=stop_at_end)
        # Разгон плавный; отказные остановки обходят этот ограничитель.
        dt = min(.15, max(0., self._dt))
        old_v, old_w = self._last_command
        vx = min(max(0., vx), speed, self.policy.max_vx, old_v + self.policy.max_accel * dt)
        wz = min(self.policy.max_wz, max(-self.policy.max_wz, wz))
        wz = min(old_w + self.policy.max_yaw_accel * dt, max(old_w - self.policy.max_yaw_accel * dt, wz))
        if vx < 1e-5:
            vx = 0.
        return self._decision(reason, vx, wz)

    def _navigate(self, obs, goal, label):
        distance = math.dist(_xy(obs.pose), goal)
        if distance <= self.policy.waypoint_tolerance:
            return None
        remaining = self._nav_path[self._nav_index:]
        if not remaining or not path_is_clear(obs.grid, [_xy(obs.pose)] + remaining, self.policy, obs.localization_error_m):
            route = astar(obs.grid, _xy(obs.pose), goal, self.policy, obs.localization_error_m)
            self._nav_path = route + [tuple(goal)] if route else []
            self._nav_index = 0
        if not self._nav_path:
            return self._wait_for_route(obs, label)
        self._resume_route(obs.t)
        if self._route_wait_total >= self.policy.route_wait_timeout:
            return self._fault("Истекло время ожидания свободного пути: " + label)
        # Навигационные точки сетки можно уплотнять; поверхность препятствия
        # обрабатывается отдельно и никогда не пропускает исходные точки.
        while self._nav_index < len(self._nav_path)-1 and math.dist(_xy(obs.pose), self._nav_path[self._nav_index]) <= self.policy.waypoint_tolerance:
            self._nav_index += 1
        # Остаток пути убывает и на необходимом обходе, где расстояние
        # по прямой до цели временно растёт. Перестроение не обновляет таймер.
        remaining = self._nav_path[self._nav_index:]
        route_distance = math.dist(_xy(obs.pose), remaining[0]) + sum(
            math.dist(a, b) for a, b in zip(remaining, remaining[1:]))
        self._mark_progress(route_distance, obs.t)
        if obs.t - self._progress_t > self.policy.progress_timeout:
            return self._fault("Нет продвижения по маршруту: " + label)
        # Узлы A* расположены через 5 см. Торможение у каждого узла держало
        # скорость ниже порога реального шага. Выбираем видимую цель дальше,
        # но только если весь прямой отрезок проверен по карте корпуса.
        target=self._nav_path[self._nav_index]
        lookahead=max(.5,self.policy.max_vx**2/(2*self.policy.max_accel)+2*self.policy.waypoint_tolerance)
        chosen = 0
        for index, candidate in enumerate(remaining[1:], 1):
            if math.dist(_xy(obs.pose),candidate)>lookahead:break
            if not path_is_clear(obs.grid,[_xy(obs.pose),candidate],self.policy,obs.localization_error_m):break
            target=candidate
            chosen=index
        # Сокращённые навигационные узлы удаляем: иначе после движения
        # к дальней цели регулятор снова потребует вернуться к узлу позади.
        if chosen:
            self._nav_path = self._nav_path[:self._nav_index] + remaining[chosen:]
        return self._drive(obs,target,self.policy.max_vx,label)

    def _support_matches(self, obs, point):
        # pose.z и body_height не используются для выдумывания высоты опоры.
        return (obs.support_verified is True and _finite(obs.support_height_m)
            and abs(obs.support_height_m - point[2]) <= min(.035, self.policy.waypoint_tolerance / 2))

    def _line_errors(self, line, obs):
        if not isinstance(line, StartLine):
            return ["Нет измеренной геометрии стартовой линии"]
        if (line.geometry_verified is not True or type(line.confirmations) is not int
                or line.confirmations < 3 or not _finite(line.confidence) or not .8 <= line.confidence <= 1.):
            return ["Геометрия стартовой линии не подтверждена"]
        if line.frame_epoch != obs.frame_epoch:
            return ["Стартовая линия в другой эпохе координат"]
        if not _finite(line.observed_at) or not 0 <= obs.t-line.observed_at <= self.policy.max_candidate_age:
            return ["Наблюдение стартовой линии устарело"]
        ends = line.endpoints
        if (not isinstance(ends,(tuple,list)) or len(ends) != 2
                or any(not isinstance(p,(tuple,list)) or len(p) != 2 or not _finite(*p) for p in ends)):
            return ["Некорректные концы стартовой линии"]
        margin = self.policy.robot_width/2+self.policy.clearance+obs.localization_error_m
        if not math.isfinite(math.dist(*ends)) or math.dist(*ends) <= 2*margin:
            return ["Подтверждённый отрезок линии слишком короток для пересечения"]
        return []

    def _observe_start_line(self, obs):
        self._return_line_problem = None
        line = obs.start_line
        if line is None or not obs.start_line_visible:
            return
        errors = self._line_errors(line,obs)
        if errors:
            self._return_line_problem = "; ".join(errors)
            return
        if self._start_line is not None:
            old = self._start_line.endpoints
            paths = (line.endpoints,tuple(reversed(line.endpoints)))
            distance,path = min(((max(math.dist(a,b) for a,b in zip(old,ends)),ends) for ends in paths),key=lambda pair:pair[0])
            if distance > self.policy.waypoint_tolerance:
                self._return_line_problem = "Изменилась привязка стартового отрезка"
                return
            # Концы не дрейфуют вслед за шумом, время и подтверждения обновляются.
            line = replace(line,endpoints=old)
        self._start_line = line

    def _return_geometry(self, obs):
        self._return_geometry_problem = "Для возврата требуется измеренная линия и подтверждённая сторона старта"
        line = self._start_line
        if line is None:
            return None
        a,b = line.endpoints
        length = math.dist(a,b)
        tangent = ((b[0]-a[0])/length,(b[1]-a[1])/length)
        normal = (-tangent[1],tangent[0])
        start_signed = sum((s-v)*n for s,v,n in zip(_xy(self.start_pose),a,normal))
        if abs(start_signed) <= obs.localization_error_m:
            return None
        home_side = 1. if start_signed > 0 else -1.
        margin = self.policy.robot_width/2+self.policy.clearance+obs.localization_error_m
        if length <= 2*margin:
            return None
        along = sum((s-v)*d for s,v,d in zip(_xy(self.start_pose),a,tangent))
        preferred = min(length-margin,max(margin,along))
        offset = max(self.policy.robot_length/2+self.policy.clearance+obs.localization_error_m,
                     2*self.policy.waypoint_tolerance+obs.localization_error_m)

        def geometry_at(along):
            center = (a[0]+along*tangent[0],a[1]+along*tangent[1])
            pre = tuple(c-home_side*n*offset for c,n in zip(center,normal))
            post = tuple(c+home_side*n*offset for c,n in zip(center,normal))
            return a,tangent,normal,length,home_side,pre,post

        if self._return_selected_along is not None:
            geometry = geometry_at(self._return_selected_along)
            if self._return_stage == "cross":
                # Во время пересечения не переносим цель на соседний участок.
                # При новом препятствии ближайший отрезок остановит _drive.
                return geometry
            remaining = self._nav_path[self._nav_index:]
            if (remaining and path_is_clear(obs.grid,[_xy(obs.pose)]+remaining,self.policy,obs.localization_error_m)
                    and path_is_clear(obs.grid,list(geometry[-2:]),self.policy,obs.localization_error_m)):
                return geometry
        # Проекция старта предпочтительна, но не является жёсткой целью.
        # Если она закрыта, ищем доступную часть ТОГО ЖЕ измеренного отрезка.
        span = length-2*margin
        count = min(256,max(1,math.ceil(span/(obs.grid.resolution*.5))))
        alternatives = [margin+span*i/count for i in range(count+1)]
        candidates = [preferred]+sorted(alternatives,key=lambda value:abs(value-preferred))
        for candidate in candidates:
            geometry = geometry_at(candidate)
            pre,post = geometry[-2:]
            if not path_is_clear(obs.grid,[pre,post],self.policy,obs.localization_error_m):
                continue
            route = astar(obs.grid,_xy(obs.pose),pre,self.policy,obs.localization_error_m)
            if route:
                self._return_selected_along = candidate
                self._nav_path = route+[pre]
                self._nav_index = 0
                return geometry
        self._return_geometry_problem = "Нет достижимого свободного участка пересечения измеренной линии"
        return None

    def _return_step(self, obs, previous_pose):
        if self.policy.return_mode=='point':
            goal=_xy(self.start_pose)
            self._return_goal=goal
            distance=math.dist(_xy(obs.pose),goal)
            arrived=distance+obs.localization_error_m<=self.policy.return_radius
            if not arrived:
                self._home_stable_since=None
                decision=self._navigate(obs,goal,'Возврат в записанную точку старта')
                return decision if decision is not None else self._decision('Уточнение положения у старта')
            if (not obs.support_verified or not _finite(obs.support_height_m,obs.speed)
                    or abs(obs.speed)>.03 or not path_is_clear(obs.grid,[_xy(obs.pose)],self.policy,obs.localization_error_m)):
                self._home_stable_since=None
                return self._decision('Ожидание остановки на подтверждённом свободном полу у старта')
            if self._home_stable_since is None:self._home_stable_since=obs.t
            if obs.t-self._home_stable_since>=self.policy.verify_seconds:
                self._phase('FINISHED',obs.t)
                return self._decision('Все требуемые препятствия пройдены; остановка в пределах допуска старта подтверждена')
            return self._decision('Подтверждение устойчивой остановки в точке старта')
        if self._return_line_problem:
            self._return_previous_crossed = bool(obs.start_line_crossed)
            self._return_crossing_armed = False
            return self._wait(obs,self._return_line_problem)
        geometry = self._return_geometry(obs)
        if geometry is None:
            return self._wait(obs,self._return_geometry_problem)
        a,tangent,normal,length,home_side,pre,post = geometry
        fresh_line = (obs.start_line_visible and obs.start_line is not None
                      and not self._line_errors(obs.start_line,obs))
        if not obs.start_line_crossed:
            self._return_crossing_armed = True
        crossing_edge = bool(obs.start_line_crossed) and not self._return_previous_crossed
        self._return_previous_crossed = bool(obs.start_line_crossed)
        geometric_crossing = False
        if previous_pose is not None:
            before = sum((p-v)*n*home_side for p,v,n in zip(_xy(previous_pose),a,normal))
            after = sum((p-v)*n*home_side for p,v,n in zip(_xy(obs.pose),a,normal))
            if before < 0 <= after:
                fraction = before/(before-after)
                point = tuple(p+fraction*(q-p) for p,q in zip(_xy(previous_pose),_xy(obs.pose)))
                along = sum((p-v)*d for p,v,d in zip(point,a,tangent))
                geometric_crossing = 0 <= along <= length
        # Подтверждается пересечение конечного отрезка к стороне старта.
        # Расстояние до записанной точки старта не заменяет геометрию линии.
        if self._return_crossing_armed and crossing_edge and fresh_line and geometric_crossing:
            self._phase("FINISHED",obs.t)
            return self._decision("Возврат через измеренный стартовый отрезок подтверждён геометрией и независимым событием")
        if self._return_stage == "approach":
            self._return_goal = pre
            decision = self._navigate(obs,pre,"Подход к измеренной стартовой линии со стороны поля")
            if decision is not None:
                return decision
            self._return_stage = "cross"
            self._return_goal = post
            self._nav_path = []
            self._nav_index = 0
            self._progress_t = obs.t
            self._best_distance = math.inf
            return self._decision("Достигнута предлиния; подготовлено пересечение к стороне старта")
        if not fresh_line:
            return self._wait(obs,"Перед пересечением требуется свежее наблюдение стартовой линии")
        self._return_goal = post
        distance = math.dist(_xy(obs.pose),post)
        self._mark_progress(distance,obs.t)
        if distance <= self.policy.waypoint_tolerance:
            return self._wait(obs,"Линия позади, но независимое событие пересечения не подтверждено")
        if obs.t-self._progress_t > self.policy.progress_timeout:
            return self._fault("Нет продвижения при пересечении стартовой линии")
        return self._drive(obs,post,min(self.policy.max_vx,.15),"Пересечение измеренного стартового отрезка")

    def step(self, obs: Observation, start=False) -> Decision:
        if self.phase == "FAULT":
            return self._decision(self._fault_reason)
        if self.phase == "FINISHED":
            if self.trial_kind:
                return self._decision('Отдельное испытание завершено; движение остановлено')
            destination = 'в точку старта' if self.policy.return_mode == 'point' else 'через стартовую линию'
            return self._decision("Все требуемые препятствия пройдены; возврат " + destination + " подтверждён")
        if not isinstance(obs, Observation) or not _finite(obs.t):
            return self._fault("Некорректное время наблюдения")
        if self.frame_epoch is not None and obs.frame_epoch != self.frame_epoch:
            return self._fault("Сменилась эпоха координат; продолжение старой миссии запрещено")
        if self.frame_epoch is not None and obs.grid is not None and obs.grid.frame_epoch != self.frame_epoch:
            return self._fault("Карта перескочила в другую эпоху координат")
        previous_t, previous_pose = self._last_t, self._last_pose
        if previous_t is not None and obs.t <= previous_t:
            self._alignment_stable_since=None
            self._verify_t = None
            self._verify_count = 0
            self._return_crossing_armed = False
            self._return_previous_crossed = bool(obs.start_line_crossed)
            self._home_stable_since=None
            return self._decision("Повторное или переставленное наблюдение; команда остановлена")
        self._dt = 0. if previous_t is None else obs.t - previous_t
        self._last_t = obs.t
        if self._started_at is not None and obs.t - self._started_at > self.policy.max_mission_seconds:
            return self._fault("Истёк предел длительности миссии")
        if self._started_at is not None and previous_t is not None and self._dt > self.policy.max_observation_age:
            self._return_crossing_armed = False
            self._return_previous_crossed = bool(obs.start_line_crossed)
            return self._wait(obs, "Разрыв свежих наблюдений; движение остановлено")
        pose_errors = self._pose_errors(obs)
        if pose_errors:
            self._return_crossing_armed = False
            self._return_previous_crossed = bool(obs.start_line_crossed)
            return self._wait(obs, "; ".join(pose_errors))
        if self._started_at is not None and previous_pose is not None and previous_t is not None:
            possible = self.policy.max_vx * self._dt + 2 * self.policy.max_localization_error + self.policy.waypoint_tolerance
            if math.dist(_xy(previous_pose), _xy(obs.pose)) > possible:
                return self._fault("Скачок положения несовместим с ограниченным движением; требуется повторная привязка")
            if abs(_angle(obs.pose.yaw - previous_pose.yaw)) > self.policy.max_wz * self._dt + .20:
                return self._fault("Скачок направления несовместим с ограниченным движением; требуется повторная привязка")
        self._last_pose = obs.pose
        identity_error = self._remember(obs)
        if identity_error:
            return self._fault(identity_error)
        self._observe_start_line(obs)
        if self.phase == "WAIT_START":
            if not start:
                return self._decision("Ожидание явного сигнала старта")
            if (self.trial_kind or self.policy.return_mode=='point') and (not obs.support_verified or not _finite(obs.support_height_m)):
                return self._decision('Для отдельной попытки требуется подтверждённая опора')
            if not self.trial_kind and self.policy.return_mode=='line' and not obs.start_line_visible:
                return self._decision("Для записи старта требуется наблюдаемая стартовая линия")
            self.start_pose = obs.pose
            self.frame_epoch = obs.frame_epoch
            self._started_at = obs.t
            self._phase("DISCOVER", obs.t)
            return self._decision("Старт и эпоха координат записаны")
        if set(self.policy.required_kinds)<=self.completed_kinds and self.phase != "RETURN":
            self.current = None
            self._nav_path = []
            self._nav_index = 0
            self._return_crossing_armed = not obs.start_line_crossed
            self._return_previous_crossed = bool(obs.start_line_crossed)
            self._return_stage = "approach"
            self._return_goal = None
            self._return_selected_along = None
            self._phase("RETURN", obs.t)
            return self._decision("Все требуемые препятствия завершены; возврат к наблюдённому старту")
        if self.current is not None:
            errors = self._refresh_target(obs)
            if errors:
                return self._wait(obs, "; ".join(errors))
        if self.phase == "DISCOVER":
            if self._select(obs):
                return self._decision("Выбран подтверждённый доступный вход")
            if self.trial_kind:
                return self._decision('Ожидание одного подтверждённого объекта '+self.trial_kind+' и свободного входа; поиск движением выключен')
            if not self._nav_path or self._nav_index >= len(self._nav_path):
                self._nav_path = frontier_path(obs.grid, obs.pose, self.policy, obs.localization_error_m) or []
                self._nav_index = 0
            if self._nav_path:
                decision = self._navigate(obs, self._nav_path[-1], "Осмотр из подтверждённой свободной области")
                if decision is not None:
                    return decision
                self._nav_path = []
            # При полной карте и отсутствии новых объектов бессмысленное
            # вращение не продолжается бесконечно: ждём наблюдение/таймаут.
            return self._wait(obs, "Нет подходящих новых объектов или достижимой свободной границы обзора")
        if self.phase == "APPROACH":
            decision = self._navigate(obs, self._approach_goal, "Подход по свободной карте")
            if decision is not None:
                return decision
            self._phase("ALIGN", obs.t)
            return self._decision("Достигнута наблюдённая зона выравнивания")
        if self.phase == "ALIGN":
            desired = _heading(self.current.path)
            error = _angle(desired - obs.pose.yaw)
            self._mark_progress(abs(error), obs.t)
            if abs(error) <= self.policy.alignment_tolerance:
                # Один кадр при пересечении нужного угла не означает, что
                # корпус уже остановился. Сначала нулевая команда и выдержка
                # по новым позам; движение по поверхности пока запрещено.
                if not obs.support_verified or not _finite(obs.support_height_m):
                    return self._wait(obs, 'Не подтверждена опора перед препятствием')
                yaw_rate = (abs(_angle(obs.pose.yaw-previous_pose.yaw))/self._dt
                            if previous_pose is not None and self._dt>0 else math.inf)
                if not _finite(obs.speed) or abs(obs.speed)>.05 or yaw_rate>self.policy.alignment_max_yaw_rate:
                    self._alignment_stable_since=None
                    return self._wait(obs, 'Ожидание остановки корпуса после выравнивания')
                if self._alignment_stable_since is None:self._alignment_stable_since=obs.t
                if obs.t-self._alignment_stable_since+1e-9 < self.policy.alignment_settle_seconds:
                    return self._decision('Проверка устойчивого направления перед проходом')
                if self.approach_only:
                    if not obs.support_verified or not _finite(obs.support_height_m):
                        return self._wait(obs, 'Не подтверждена опора перед препятствием')
                    if not _finite(obs.speed) or abs(obs.speed) > .05:
                        return self._decision('Остановка перед следующим препятствием')
                    self._phase('FINISHED', obs.t)
                    return self._decision('Подход и выравнивание завершены; ожидается отдельная кнопка прохождения')
                self.waypoint_index = 0
                self.visited_waypoints = []
                self._phase("TRAVERSE", obs.t)
                return self._decision("Направление входа согласовано; прохождение по всем точкам поверхности")
            self._alignment_stable_since=None
            if obs.t - self._progress_t > self.policy.progress_timeout:
                return self._fault("Нет продвижения при выравнивании")
            if not obs.support_verified or not _finite(obs.support_height_m):
                return self._wait(obs, "Не подтверждена опора в зоне выравнивания")
            wz = max(-self.policy.max_wz, min(self.policy.max_wz, 1.5 * error))
            change = self.policy.max_yaw_accel * min(.15, self._dt)
            wz = min(self._last_command[1] + change, max(self._last_command[1] - change, wz))
            return self._decision("Выравнивание внутри проверенного габарита", 0., wz)
        if self.phase == "TRAVERSE":
            path = self.current.path
            point = path[self.waypoint_index]
            if not obs.support_verified or not _finite(obs.support_height_m):
                return self._wait(obs, "Нет независимой подтверждённой высоты опоры")
            distance = math.dist(_xy(obs.pose), point[:2])
            self._mark_progress(distance + abs(obs.support_height_m - point[2]), obs.t)
            while distance <= self.policy.waypoint_tolerance and self._support_matches(obs, point):
                self.visited_waypoints.append((self.waypoint_index, obs.t, obs.pose.x, obs.pose.y, obs.support_height_m))
                if self.waypoint_index == 0:
                    self._entry_pose = obs.pose
                self.waypoint_index += 1
                self._progress_t = obs.t
                self._best_distance = math.inf
                if self.waypoint_index == len(path):
                    self._phase("VERIFY", obs.t)
                    self._verify_t = None
                    self._verify_count = 0
                    return self._decision("Все точки поверхности посещены; проверяется физический выход")
                # Внутренний узел не требует StopMove. Все посещения всё равно
                # подтверждены текущими координатами и измеренной высотой опоры.
                point = path[self.waypoint_index]
                distance = math.dist(_xy(obs.pose), point[:2])
            if self.waypoint_index > 0:
                a, b = path[self.waypoint_index-1], point
                dx, dy = b[0]-a[0], b[1]-a[1]
                norm2 = dx*dx+dy*dy
                projection = ((obs.pose.x-a[0])*dx+(obs.pose.y-a[1])*dy) / norm2 if norm2 > 1e-10 else 0.
                future_reached = any(math.dist(_xy(obs.pose), q[:2]) <= self.policy.waypoint_tolerance
                    and self._support_matches(obs, q) for q in path[self.waypoint_index+1:])
                if projection > 1 + self.policy.waypoint_tolerance / max(math.sqrt(norm2), 1e-6) or (future_reached and distance > self.policy.waypoint_tolerance):
                    return self._fault("Пропущена обязательная точка поверхности; зачёт запрещён")
                # Коридор опоры должен удерживаться и между обязательными точками.
                if norm2 > 1e-10:
                    nearest = (a[0]+min(1.,max(0.,projection))*dx, a[1]+min(1.,max(0.,projection))*dy)
                    radius = self.policy.robot_width/2
                    if self.current.id in obs.traversal_grids:
                        radius = math.hypot(self.policy.robot_length,self.policy.robot_width)/2
                    allowance = self.current.width/2 - radius - self.policy.clearance - obs.localization_error_m
                    if math.dist(_xy(obs.pose), nearest) > max(0., allowance):
                        return self._fault("Отклонение от подтверждённого коридора поверхности")
            if obs.t - self._progress_t > self.policy.progress_timeout:
                return self._fault("Нет физического продвижения по поверхности")
            speed = self.policy.speed_for_skill(self.current.kind)
            return self._drive(obs, point[:2], speed, "Прохождение " + self.current.kind + ": точка " + str(self.waypoint_index),
                               stop_at_end=self.waypoint_index == len(path)-1)
        if self.phase == "VERIFY":
            item = self.current
            exit_goal = self._exit_goal or item.path[-1][:2]
            exit_distance = math.dist(_xy(obs.pose),exit_goal)
            self._mark_progress(exit_distance,obs.t)
            if exit_distance > self.policy.waypoint_tolerance:
                if not obs.support_verified or not _finite(obs.support_height_m):
                    return self._wait(obs,"Не подтверждена опора при выходе на пол")
                if obs.t-self._progress_t > self.policy.progress_timeout:
                    return self._fault("Нет физического выхода всем габаритом на подтверждённый пол")
                return self._drive(obs,exit_goal,min(self.policy.traverse_speed,.10),"Выход с поверхности на подтверждённый пол")
            at_exit = (self._support_matches(obs,item.path[-1])
                and path_is_clear(obs.grid,[_xy(obs.pose)],self.policy,obs.localization_error_m))
            expected = math.dist(item.path[0][:2], item.path[-1][:2])
            moved = self._entry_pose is not None and math.dist(_xy(self._entry_pose), _xy(obs.pose)) + 1e-8 >= max(
                2*self.policy.waypoint_tolerance, expected-2*self.policy.waypoint_tolerance)
            visited = len(self.visited_waypoints) == len(item.path)
            stable = _finite(obs.speed) and abs(obs.speed) <= .05
            if not (at_exit and moved and visited and stable):
                self._verify_t = None
                self._verify_count = 0
                return self._wait(obs, "Выход не подтверждён положением, опорой, полным путём и остановкой")
            if self._verify_t is None:
                self._verify_t = obs.t
            self._verify_count += 1
            if self._verify_count < 3 or obs.t - self._verify_t < self.policy.verify_seconds:
                return self._decision("Выход физически достигнут; подтверждается устойчивость")
            if item.id in self.completed_ids or item.kind in self.completed_kinds:
                return self._fault("Повторный зачёт объекта или вида запрещён")
            self.completed_ids.add(item.id)
            self.completed_kinds.add(item.kind)
            self.completion_order.append(item.kind)
            if self.trial_kind:
                self.current = None
                self._phase('FINISHED', obs.t)
                return self._decision('Отдельное препятствие: путь, выход и устойчивость подтверждены')
            self.current = None
            self._nav_path = []
            self._nav_index = 0
            self._phase("DISCOVER", obs.t)
            return self._decision("Прохождение подтверждено; вид засчитан один раз")
        if self.phase == "RETURN":
            return self._return_step(obs,previous_pose)
        return self._fault("Неизвестное состояние автомата")
