"""Планирование только по подтверждённому свободному пространству.

Модуль не обращается к роботу. Маска сохраняет запас для всего корпуса
при любом yaw, поэтому допустима и остановка с разворотом на месте.
"""
from collections import deque
import heapq
import math

from .models import Grid, Obstacle, Policy, Pose


_EPS = 1e-10
_NEIGHBORS = ((1, 0), (-1, 0), (0, 1), (0, -1),
              (1, 1), (1, -1), (-1, 1), (-1, -1))


def _finite_point(point):
    return (len(point) >= 2 and all(isinstance(v, (int, float))
            and not isinstance(v, bool) and math.isfinite(v) for v in point))


def _shape(grid):
    try:
        h = len(grid.cells)
        w = len(grid.cells[0]) if h else 0
        return (w, h) if w and all(len(row) == w for row in grid.cells) else (0, 0)
    except (AttributeError, TypeError):
        return 0, 0


def _radius(policy, uncertainty):
    policy.validate()
    if (isinstance(uncertainty, bool) or not isinstance(uncertainty, (int, float))
            or not math.isfinite(uncertainty) or uncertainty < 0
            or uncertainty > policy.max_localization_error):
        raise ValueError('Не подтверждена точность локализации')
    return math.hypot(policy.robot_length, policy.robot_width) / 2 + policy.clearance + uncertainty


def safe_mask(grid: Grid, policy: Policy, localization_error: float) -> list[list[bool]]:
    """Свободна вся ячейка для описанной вокруг корпуса окружности.

    Радиус = hypot(length,width)/2 + clearance + localization_error.
    Расстояние считается между квадратами ячеек, а не только центрами:
    промежуточные точки пути и любые повороты также сохраняют запас.
    Неизвестность, край карты и неподтверждённая карта закрыты.
    """
    w, h = _shape(grid)
    mask = [[False] * w for _ in range(h)]
    try:
        radius = _radius(policy, localization_error)
        if (not w or grid.verified is not True or not _finite_point(grid.origin)
                or len(grid.origin) != 2 or isinstance(grid.resolution, bool)
                or not math.isfinite(grid.resolution) or grid.resolution <= 0
                or any(type(v) is not int or v not in (-1, 0, 1)
                       for row in grid.cells for v in row)):
            return mask
    except (AttributeError, TypeError, ValueError):
        return mask
    res = grid.resolution
    for y in range(h):
        for x in range(w):
            mask[y][x] = (grid.cells[y][x] == 0
                          and min(x, y, w - x - 1, h - y - 1) * res > radius + _EPS)
    # Интегральная сумма быстро исключает дальние препятствия и неизвестность.
    # В кольце рядом с ними проверяются строки окружности, а не все её клетки.
    sums = [[0] * (w + 1) for _ in range(h + 1)]
    for y, row in enumerate(grid.cells):
        total = 0
        for x, value in enumerate(row):
            total += value != 0
            sums[y + 1][x + 1] = sums[y][x + 1] + total

    def blocked(left, top, right, bottom):
        left, top, right, bottom = max(0, left), max(0, top), min(w, right), min(h, bottom)
        return sums[bottom][right] - sums[top][right] - sums[bottom][left] + sums[top][left]

    reach = int(math.ceil(radius / res)) + 1
    spans = []
    for dy in range(-reach, reach + 1):
        gap = max(abs(dy) - 1, 0) * res
        if gap <= radius + _EPS:
            dx = math.floor(math.sqrt(max(0., (radius + _EPS) ** 2 - gap ** 2)) / res + 1)
            spans.append((dy, dx))
    spans.sort(key=lambda item: abs(item[0]))
    for y in range(h):
        for x in range(w):
            if not mask[y][x] or not blocked(x - reach, y - reach, x + reach + 1, y + reach + 1):
                continue
            for dy, dx in spans:
                if 0 <= y + dy < h and blocked(x - dx, y + dy, x + dx + 1, y + dy + 1):
                    mask[y][x] = False
                    break
    return mask


def _cell(grid, point):
    return (math.floor((point[0] - grid.origin[0]) / grid.resolution),
            math.floor((point[1] - grid.origin[1]) / grid.resolution))


def _center(grid, cell):
    return (grid.origin[0] + (cell[0] + .5) * grid.resolution,
            grid.origin[1] + (cell[1] + .5) * grid.resolution)


def _allowed(mask, cell):
    x, y = cell
    return bool(mask and 0 <= y < len(mask) and 0 <= x < len(mask[0]) and mask[y][x])


def _touching_cells(grid, point):
    """Точка на границе принадлежит обеим ячейкам, в углу — четырём."""
    axes = []
    for p, origin in zip(point[:2], grid.origin):
        value = (p - origin) / grid.resolution
        rounded = round(value)
        axes.append((rounded - 1, rounded) if abs(value - rounded) <= 1e-9
                    else (math.floor(value),))
    return ((x, y) for x in axes[0] for y in axes[1])


def _segment_clear(grid, mask, a, b):
    # Разбиение в точных пересечениях сетки исключает пропущенные тонкие углы.
    events = [0.0, 1.0]
    for axis in (0, 1):
        delta = b[axis] - a[axis]
        if abs(delta) <= _EPS:
            continue
        low, high = sorted(((a[axis] - grid.origin[axis]) / grid.resolution,
                            (b[axis] - grid.origin[axis]) / grid.resolution))
        for boundary in range(math.ceil(low), math.floor(high) + 1):
            t = (grid.origin[axis] + boundary * grid.resolution - a[axis]) / delta
            if 0 < t < 1:
                events.append(t)
    events = sorted(set(events))
    samples = events + [(lo + hi) / 2 for lo, hi in zip(events, events[1:])]
    return all(_allowed(mask, cell) for t in samples
               for cell in _touching_cells(grid, (a[0] + t * (b[0] - a[0]),
                                                  a[1] + t * (b[1] - a[1]))))


def _path_clear(grid, mask, points):
    if not points or not all(_finite_point(p) for p in points):
        return False
    if not all(_allowed(mask, cell) for p in points for cell in _touching_cells(grid, p)):
        return False
    return all(_segment_clear(grid, mask, a, b) for a, b in zip(points, points[1:]))


def path_is_clear(grid: Grid, points_xy, policy: Policy, uncertainty: float) -> bool:
    """Проверяет непрерывную ломаную, включая весь корпус и неизвестность."""
    try:
        mask = safe_mask(grid, policy, uncertainty)
        return bool(mask and any(map(any, mask)) and _path_clear(grid, mask, points_xy))
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False


def _neighbors(mask, cell):
    x, y = cell
    for dx, dy in _NEIGHBORS:
        neighbor = x + dx, y + dy
        if not _allowed(mask, neighbor):
            continue
        if dx and dy and (not _allowed(mask, (x + dx, y))
                          or not _allowed(mask, (x, y + dy))):
            continue
        yield neighbor, math.hypot(dx, dy)


def _reconstruct(grid, parents, goal):
    cells = [goal]
    while cells[-1] in parents:
        cells.append(parents[cells[-1]])
    return [_center(grid, cell) for cell in reversed(cells)]


def _astar(grid, start_xy, goal_xy, mask):
    if not _path_clear(grid, mask, [start_xy]) or not _path_clear(grid, mask, [goal_xy]):
        return None
    start, goal = _cell(grid, start_xy), _cell(grid, goal_xy)
    costs, parents = {start: 0.0}, {}
    queue = [(math.dist(start, goal), 0.0, start)]
    while queue:
        _, cost, cell = heapq.heappop(queue)
        if cost > costs[cell] + _EPS:
            continue
        if cell == goal:
            return _reconstruct(grid, parents, goal)
        for neighbor, step in _neighbors(mask, cell):
            candidate = cost + step
            if candidate + _EPS < costs.get(neighbor, math.inf):
                costs[neighbor], parents[neighbor] = candidate, cell
                heapq.heappush(queue, (candidate + math.dist(neighbor, goal), candidate, neighbor))
    return None


def astar(grid: Grid, start_xy, goal_xy, policy: Policy, localization_error: float):
    """Восемь соседей, запрет срезания углов; возвращает центры ячеек.

    Реальные start/goal тоже должны быть безопасны; их ячейки включены.
    Конечная точка никогда не подменяется ближайшей свободной ячейкой.
    """
    try:
        mask = safe_mask(grid, policy, localization_error)
        return _astar(grid, start_xy, goal_xy, mask) if mask and any(map(any, mask)) else None
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def follow_path(pose: Pose, path, policy: Policy, speed_limit=None, stop_at_end=True):
    """Команда для первого недостигнутого узла, без скачка через повороты.

    Вызывающий FSM обязан проверить текущую позицию и путь path_is_clear
    и удалить уже пройденный префикс. Без grid эта функция не может
    самостоятельно подтвердить место для разворота. Обратный ход запрещён.
    """
    try:
        policy.validate()
        if not path or not _finite_point((pose.x, pose.y, pose.yaw)) or not all(_finite_point(p) for p in path):
            return 0.0, 0.0
        if speed_limit is not None and (isinstance(speed_limit, bool)
                or not math.isfinite(speed_limit) or speed_limit <= 0):
            return 0.0, 0.0
        limit = policy.max_vx if speed_limit is None else min(policy.max_vx, speed_limit)
        if isinstance(limit, bool) or not math.isfinite(limit) or limit <= 0:
            return 0.0, 0.0
        index = 0
        while index < len(path) and math.dist((pose.x, pose.y), path[index][:2]) <= policy.waypoint_tolerance:
            index += 1
        if index == len(path):
            return 0.0, 0.0
        target = path[index]
        distance = math.hypot(target[0] - pose.x, target[1] - pose.y)
        bearing = math.atan2(target[1] - pose.y, target[0] - pose.x)
        error = _wrap(bearing - pose.yaw)
        wz = max(-policy.max_wz, min(policy.max_wz, 1.5 * error))
        if abs(error) > max(.35, 3 * policy.alignment_tolerance):
            return 0.0, wz
        speed = limit * math.cos(error) ** 2
        curvature = 2 * abs(math.sin(error)) / max(distance, policy.waypoint_tolerance)
        if curvature > _EPS:
            speed = min(speed, policy.max_wz / curvature, math.sqrt(policy.max_accel / curvature))
        sharp_turn = False
        if index + 1 < len(path):
            after = path[index + 1]
            outgoing = math.atan2(after[1] - target[1], after[0] - target[0])
            sharp_turn = abs(_wrap(outgoing - bearing)) > policy.alignment_tolerance
        if sharp_turn or (stop_at_end and index == len(path) - 1):
            speed = min(speed, math.sqrt(2 * policy.max_accel * max(0.0, distance - policy.waypoint_tolerance)))
        return max(0.0, speed), wz
    except (AttributeError, TypeError, ValueError, OverflowError):
        return 0.0, 0.0


def frontier_path(grid: Grid, pose: Pose, policy: Policy, uncertainty: float):
    """Подходит к границе наблюдений, оставаясь всем корпусом в известном.

    Фронтир — известная клетка рядом с неизвестной. Цель выбирается перед
    ним с учётом габаритов; стены не используются как фронтиры.
    """
    mask = safe_mask(grid, policy, uncertainty)
    if not mask or not any(map(any, mask)) or not path_is_clear(grid, [(pose.x, pose.y)], policy, uncertainty):
        return None
    w, h = _shape(grid)
    frontier_distance, wave = {}, deque()
    for y in range(h):
        for x in range(w):
            if grid.cells[y][x] == 0 and any(0 <= x + dx < w and 0 <= y + dy < h
                    and grid.cells[y + dy][x + dx] == -1 for dx, dy in _NEIGHBORS[:4]):
                frontier_distance[(x, y)] = 0
                wave.append((x, y))
    extent = int(math.ceil(_radius(policy, uncertainty) / grid.resolution)) + 3
    while wave:
        x, y = wave.popleft()
        distance = frontier_distance[(x, y)]
        if distance >= extent:
            continue
        for dx, dy in _NEIGHBORS[:4]:
            n = x + dx, y + dy
            if 0 <= n[0] < w and 0 <= n[1] < h and grid.cells[n[1]][n[0]] == 0 and n not in frontier_distance:
                frontier_distance[n] = distance + 1
                wave.append(n)
    start = _cell(grid, (pose.x, pose.y))
    parents, costs, queue = {}, {start: 0.0}, [(0.0, start)]
    candidates = []
    while queue:
        cost, cell = heapq.heappop(queue)
        if cost > costs[cell] + _EPS:
            continue
        if (cell in frontier_distance and math.dist(_center(grid, cell), (pose.x, pose.y))
                >= max(2 * grid.resolution, 2 * policy.waypoint_tolerance)):
            candidates.append((frontier_distance[cell] + .15 * cost, cost, cell))
        for neighbor, step in _neighbors(mask, cell):
            candidate = cost + step
            if candidate + _EPS < costs.get(neighbor, math.inf):
                costs[neighbor], parents[neighbor] = candidate, cell
                heapq.heappush(queue, (candidate, neighbor))
    if not candidates:
        return None
    goal = min(candidates)[2]
    return _reconstruct(grid, parents, goal)


def _length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def _yaw(points, last=False):
    pairs = list(zip(points, points[1:]))
    if last:
        pairs.reverse()
    for a, b in pairs:
        if math.dist(a[:2], b[:2]) > _EPS:
            return math.atan2(b[1] - a[1], b[0] - a[0])
    raise ValueError('Поверхность не имеет направления')


def optimize_route(pose: Pose, obstacles: list[Obstacle], grid: Grid, policy: Policy,
                   uncertainty: float, *, home: Pose | None = None,
                   surface_grids: dict[str, Grid] | None = None,
                   stages: dict[tuple[str, bool], tuple[tuple[float, float], tuple[float, float]]] | None = None
                   ) -> list[tuple[str, bool]]:
    """Точный поиск порядка и допустимых ориентаций для не более пяти целей.

    Между наблюдёнными поверхностями стоимость и достижимость берутся из
    A* по исходной карте пола. Путь каждого препятствия и соединения с его
    входной/выходной площадками проверяются только в его surface_grid.
    Карты поверхностей никогда не объединяются для транзита между ними.
    Без stages площадки совпадают с концами пути, как в исходном API;
    при заданном stages отсутствие ключа запрещает данную ориентацию.
    Учитываются длина поверхностей и развороты, при home — возврат.
    Если полный маршрут невозможен, возвращается []; нет жадной подмены.
    """
    if len(obstacles) > 5:
        raise ValueError('Точный план поддерживает не более пяти препятствий')
    if not obstacles:
        return []
    mask = safe_mask(grid, policy, uncertainty)
    if not mask or not any(map(any, mask)):
        return []
    try:
        if not _finite_point((pose.x, pose.y, pose.yaw)) or (home is not None and not _finite_point((home.x, home.y, home.yaw))):
            return []
        if len({ob.id for ob in obstacles}) != len(obstacles):
            return []
        options = []
        turn_scale = policy.max_vx / policy.max_wz
        for index, obstacle in enumerate(obstacles):
            if (not obstacle.geometry_verified or not obstacle.direction_verified
                    or obstacle.frame_epoch != grid.frame_epoch or len(obstacle.path) < 2
                    or not all(_finite_point(p) and len(p) == 3 for p in obstacle.path)):
                return []
            surface_grid = grid if surface_grids is None else surface_grids.get(obstacle.id, grid)
            if (surface_grid.frame_epoch != grid.frame_epoch or _shape(surface_grid) != _shape(grid)
                    or tuple(surface_grid.origin) != tuple(grid.origin)
                    or surface_grid.resolution != grid.resolution):
                return []
            surface_mask = mask if surface_grid is grid else safe_mask(surface_grid, policy, uncertainty)
            if (not surface_mask or not any(map(any, surface_mask))
                    or any(original == -1 and overlay == 0
                           for floor_row, surface_row in zip(grid.cells, surface_grid.cells)
                           for original, overlay in zip(floor_row, surface_row))
                    or not _path_clear(surface_grid, surface_mask, obstacle.path)):
                return []
            for reverse in ([False, True] if obstacle.bidirectional else [False]):
                path = list(reversed(obstacle.path)) if reverse else list(obstacle.path)
                if stages is None:
                    entry, exit_point = path[0][:2], path[-1][:2]
                elif (obstacle.id, reverse) in stages:
                    stage = stages[(obstacle.id, reverse)]
                    if len(stage) != 2 or any(not _finite_point(p) or len(p) != 2 for p in stage):
                        continue
                    entry, exit_point = stage
                else:
                    continue
                if (not _path_clear(grid, mask, [entry]) or not _path_clear(grid, mask, [exit_point])
                        or not _path_clear(surface_grid, surface_mask, [entry, path[0][:2]])
                        or not _path_clear(surface_grid, surface_mask, [path[-1][:2], exit_point])):
                    continue
                points = [entry] + [p[:2] for p in path] + [exit_point]
                yaws = [math.atan2(b[1] - a[1], b[0] - a[0])
                        for a, b in zip(points, points[1:]) if math.dist(a, b) > _EPS]
                if not yaws:
                    continue
                surface_cost = (_length(path) + math.dist(entry, path[0][:2])
                                + math.dist(path[-1][:2], exit_point)
                                + turn_scale * sum(abs(_wrap(b - a)) for a, b in zip(yaws, yaws[1:])))
                options.append((index, reverse, entry, exit_point, yaws[0], yaws[-1], surface_cost))
            if not any(option[0] == index for option in options):
                return []
        cache = {}

        def transit(start, start_yaw, goal, goal_yaw):
            key = tuple(start[:2]), start_yaw, tuple(goal[:2]), goal_yaw
            if key not in cache:
                path = _astar(grid, start[:2], goal[:2], mask)
                if path is None:
                    cache[key] = math.inf
                else:
                    points = [start[:2]] + path + [goal[:2]]
                    yaws = [math.atan2(b[1] - a[1], b[0] - a[0])
                            for a, b in zip(points, points[1:]) if math.dist(a, b) > _EPS]
                    angles = [start_yaw] + yaws + [goal_yaw]
                    cache[key] = _length(points) + turn_scale * sum(abs(_wrap(b - a)) for a, b in zip(angles, angles[1:]))
            return cache[key]

        # Динамическое программирование перебирает все перестановки/ориентации.
        states = {(0, -1): (0.0, [])}
        for count in range(len(obstacles)):
            for (visited, previous), (cost, route) in list(states.items()):
                if visited.bit_count() != count:
                    continue
                if previous < 0:
                    start, start_yaw = (pose.x, pose.y), pose.yaw
                else:
                    _, _, _, start, _, start_yaw, _ = options[previous]
                for option_index, (index, reverse, entry, _, entry_yaw, _, surface_cost) in enumerate(options):
                    if visited & (1 << index):
                        continue
                    extra = transit(start, start_yaw, entry, entry_yaw) + surface_cost
                    candidate = cost + extra
                    key = visited | (1 << index), option_index
                    if math.isfinite(candidate) and candidate < states.get(key, (math.inf, []))[0] - _EPS:
                        states[key] = candidate, route + [(obstacles[index].id, reverse)]
        complete = []
        full = (1 << len(obstacles)) - 1
        for (visited, previous), (cost, route) in states.items():
            if visited != full:
                continue
            if home is not None:
                _, _, _, exit_point, _, exit_yaw, _ = options[previous]
                cost += transit(exit_point, exit_yaw, (home.x, home.y), home.yaw)
            if math.isfinite(cost):
                complete.append((cost, route))
        return min(complete)[1] if complete else []
    except (AttributeError, TypeError, ValueError, OverflowError):
        return []
