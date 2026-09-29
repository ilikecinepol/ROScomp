"""Наблюдения из метрических поверхностей и BGR без команд роботу.

Точки занятости не являются свободными лучами. Проходом становится только
наблюдённая плоскость под всей расширенной опорой; пропуски остаются unknown.
Кадр камеры даёт диагностические пиксельные кандидаты без выдуманной K.
"""
from __future__ import annotations

import math
import heapq
import numpy as np
from scipy import ndimage

from .models import Grid, Obstacle, Observation, Policy, Pose


def _contract_errors(contract, pose, epoch, policy):
    c = contract if isinstance(contract, dict) else {}
    errors = []
    for key in ('geometry_frames_validated', 'source_time_validated', 'identity_verified'):
        if c.get(key) is not True:
            errors.append('Не подтверждено профилем/монитором: ' + key)
    if c.get('units') != 'm' or c.get('up_axis') != '+z':
        errors.append('Нужны явные метрические единицы и ось +z вверх')
    if not isinstance(c.get('point_frame'), str) or not c['point_frame'].strip() or c.get('point_frame') != c.get('pose_frame'):
        errors.append('Системы точек и позы не подтверждены как одна система')
    if not isinstance(c.get('pose_child_frame'), str) or not c['pose_child_frame'].strip():
        errors.append('Не указан проверенный pose_child_frame')
    if c.get('source_kind') not in ('observed_surface_points', 'occupied_voxel_points'):
        errors.append('Не указан поддерживаемый источник наблюдённых поверхностей')
    if c.get('source_kind') == 'occupied_voxel_points':
        size = c.get('voxel_size_m')
        if not isinstance(size, (int, float)) or isinstance(size, bool) or not math.isfinite(size) or not .005 <= size <= .1:
            errors.append('Для занятого вокселя нужен проверенный voxel_size_m')
        if c.get('voxel_reference') not in ('lower_corner', 'center'):
            errors.append('Не определено, что означает точка вокселя: lower_corner или center')
    for key, limit in (('acquisition_age_s', policy.max_observation_age), ('localization_error_m', policy.max_localization_error)):
        value = c.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= limit:
            errors.append('Недопустимо или неизвестно: ' + key)
    if type(epoch) is not int or epoch < 0 or c.get('frame_epoch', epoch) != epoch:
        errors.append('Контракт принадлежит другой эпохе координат')
    if not isinstance(pose, Pose) or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in (pose.x, pose.y, pose.z, pose.yaw)):
        errors.append('Нет конечной позы')
    return errors


def camera_candidates(image):
    """Цветовые конусы, четырёхугольные скаты и углы: только пиксели."""
    if image is None:
        return []
    try:
        import cv2
    except ImportError:
        return [{'kind': 'camera_unavailable', 'reason': 'Для BGR анализа нужен OpenCV'}]
    bgr = np.asarray(image)
    if bgr.ndim != 3 or bgr.shape[2] != 3 or min(bgr.shape[:2]) < 4 or bgr.dtype != np.uint8:
        return [{'kind': 'invalid_image', 'reason': 'Нужен uint8 BGR HxWx3'}]
    height, width = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    orange = cv2.inRange(hsv, (3, 95, 70), (28, 255, 255))
    result = []
    for contour in cv2.findContours(orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        x, y, w, h = cv2.boundingRect(contour)
        if cv2.contourArea(contour) >= max(12, width*height*.0001) and h > w*.75:
            result.append({'kind': 'cone_image_candidate', 'bbox_px': [x, y, w, h], 'metric_target': False,
                           'reason': 'Оранжевый силуэт; цвет не доказывает тип предмета'})
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 45, 130)
    for contour in cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]:
        area = cv2.contourArea(contour)
        if not .007*width*height < area < .5*width*height:
            continue
        poly = cv2.approxPolyDP(contour, .035*cv2.arcLength(contour, True), True)
        if len(poly) != 4 or not cv2.isContourConvex(poly):
            continue
        x, y, w, h = cv2.boundingRect(poly)
        if h < 25 or w < 25:
            continue
        lines = cv2.HoughLinesP(edges[y:y+h, x:x+w], 1, np.pi/180, 20, minLineLength=w*.35, maxLineGap=8)
        transverse = 0 if lines is None else sum(abs(line[3]-line[1]) < .3*abs(line[2]-line[0]) for line in np.asarray(lines).reshape(-1, 4))
        if transverse >= 2:
            result.append({'kind': 'ramp_image_candidate', 'bbox_px': [x, y, w, h],
                           'corners_px': poly.reshape(-1, 2).tolist(), 'transverse_segments': int(transverse),
                           'metric_target': False, 'reason': 'Четырёхугольник с поперечными границами; угол наклона и масштаб неизвестны'})
    corners = cv2.goodFeaturesToTrack(gray, 25, .05, max(8, width*.025))
    if corners is not None:
        result.append({'kind': 'image_corners', 'points_px': corners.reshape(-1, 2).tolist(), 'metric_target': False})
    return result[:30]


def _raster(points, resolution):
    origin = points[:, :2].min(axis=0)
    # Привязка к наблюдаемой решётке сохраняет полувоксельный сдвиг центров;
    # банковское округление точек x=n+.5 иначе создало бы ложные дыры.
    ij = np.floor((points[:, :2]-origin)/resolution+.5+1e-8).astype(int)
    shape = ij.max(axis=0)+1
    if np.any(shape > 500) or np.prod(shape) > 150000:
        raise ValueError('Слишком большая локальная карта; нужен ограниченный наблюдаемый участок')
    top = np.full((shape[1], shape[0]), -np.inf)
    np.maximum.at(top, (ij[:, 1], ij[:, 0]), points[:, 2])
    top[~np.isfinite(top)] = np.nan
    yy, xx = np.indices(top.shape)
    return top, origin, origin[0]+xx*resolution, origin[1]+yy*resolution, ij


def _floor(top, xx, yy, pose, resolution):
    valid = np.isfinite(top)
    near = valid & (np.hypot(xx-pose.x, yy-pose.y) < 1.5) & (top <= pose.z+.05)
    if near.sum() < 30:
        return None
    points = np.column_stack([xx[near], yy[near], top[near]])
    rng = np.random.default_rng(3)
    matrix = np.column_stack([points[:, :2], np.ones(len(points))])
    best = None
    for _ in range(90):
        sample = rng.choice(len(points), 3, replace=False)
        if abs(np.linalg.det(matrix[sample])) < .01:
            continue
        coef = np.linalg.solve(matrix[sample], points[sample, 2])
        if np.linalg.norm(coef[:2]) > math.tan(math.radians(8)):
            continue
        # Низкие края ската не должны наклонять оценку горизонтального пола.
        mask = abs(matrix@coef-points[:, 2]) <= max(.004, resolution*.15)
        if best is None or mask.sum() > best.sum():
            best = mask
    if best is None or best.sum() < 30:
        return None
    coef = np.linalg.lstsq(matrix[best], points[best, 2], rcond=None)[0]
    return coef, coef[0]*xx+coef[1]*yy+coef[2]


def _support_grid(top, floor_height, origin, resolution, epoch, policy, voxel_size=0.):
    observed = np.isfinite(top)
    tolerance = max(.012, resolution*.45)
    # Высота занятого вокселя — интервал, а не точное измерение поверхности.
    # Соседние уровни могут представлять один пол. Расширение допуска возможно
    # лишь когда полный перепад двух интервалов укладывается в предел ступени.
    if not math.isfinite(voxel_size) or voxel_size < 0:
        raise ValueError('Некорректная погрешность высоты вокселя')
    if voxel_size and 2*voxel_size <= policy.max_step:
        tolerance = max(tolerance, voxel_size)
    floor_samples = observed & (abs(top-floor_height) <= tolerance+1e-8)
    cells = np.full(top.shape, -1, dtype=np.int8)
    # Grid хранит наблюдения, а navigation.safe_mask учитывает габариты один раз.
    cells[floor_samples] = 0
    raised = observed & (top-floor_height > max(.025, resolution*.6,tolerance)+1e-8)
    cells[observed & ~floor_samples] = 1
    return Grid(tuple(origin-resolution/2), resolution, cells.tolist(), epoch, True), raised


def _supported(grid, xy, safe=None):
    ix = int(math.floor((xy[0]-grid.origin[0])/grid.resolution))
    iy = int(math.floor((xy[1]-grid.origin[1])/grid.resolution))
    return 0 <= iy < len(grid.cells) and 0 <= ix < len(grid.cells[0]) and (safe[iy][ix] if safe is not None else grid.cells[iy][ix] == 0)


def _local_support(top, xx, yy, pose, resolution, policy, contract):
    """Высота наблюдённой поверхности под позой, отдельно от глобального пола.

    Полный ориентированный footprint должен наблюдаться в текущем облаке.
    Это геометрия поверхности, а не датчик контакта лап или силы опоры.
    """
    uncertainty = float(contract['localization_error_m'])
    half_length = policy.robot_length/2+policy.clearance+uncertainty
    half_width = policy.robot_width/2+policy.clearance+uncertainty
    co, si = math.cos(pose.yaw), math.sin(pose.yaw)
    corners = np.array([[pose.x+co*u-si*v, pose.y+si*u+co*v]
                        for u in (-half_length, half_length) for v in (-half_width, half_width)])
    if corners[:, 0].min() < xx.min() or corners[:, 0].max() > xx.max() or corners[:, 1].min() < yy.min() or corners[:, 1].max() > yy.max():
        return None, 'Опора выходит за наблюдаемую карту'
    dx, dy = xx-pose.x, yy-pose.y
    mask = (abs(co*dx+si*dy) <= half_length+resolution*.71) & (abs(-si*dx+co*dy) <= half_width+resolution*.71)
    if mask.sum() < 9 or not np.isfinite(top[mask]).all():
        return None, 'Часть footprint не наблюдается; высота опоры не подтверждена'
    if np.max(top[mask]) >= pose.z-.02:
        return None, 'В footprint есть поверхность на высоте корпуса или выше'
    deltas = []
    for axis in (0, 1):
        one = (slice(None, -1), slice(None)) if axis == 0 else (slice(None), slice(None, -1))
        two = (slice(1, None), slice(None)) if axis == 0 else (slice(None), slice(1, None))
        paired = mask[one] & mask[two]
        deltas.extend(abs(top[two][paired]-top[one][paired]).tolist())
    deltas = np.asarray(deltas)
    slope_step = math.tan(math.radians(policy.max_slope_deg))*resolution+.005
    if not len(deltas) or np.max(deltas) > policy.max_step+.005 or np.mean(deltas > slope_step) > .15:
        return None, 'Перепады внутри footprint не образуют допустимую наблюдённую опору'
    ix = int(np.floor((pose.x-xx[0, 0])/resolution+.5))
    iy = int(np.floor((pose.y-yy[0, 0])/resolution+.5))
    height = float(top[iy, ix])
    # Sport body_height нельзя трактовать как смещение base_link без отдельной
    # проверки его физического смысла на данном устройстве.
    if contract.get('body_height_reference_verified') is True and contract.get('body_height_reference_evidence'):
        body_height = contract.get('body_height_m')
        if not isinstance(body_height, (int, float)) or isinstance(body_height, bool) or not math.isfinite(body_height) or not policy.minimum_body_height <= body_height <= policy.max_body_height:
            return None, 'Нет свежей высоты корпуса для проверенного reference'
        if abs(height-(pose.z-body_height)) > .035+uncertainty:
            return None, 'Наблюдаемая поверхность не совпадает с независимо проверенной высотой опоры'
    return height, None


def _route_on_support(grid, safe, start, goal):
    """Соединяет боковые точки слалома по уже проверенной опоре корпуса."""
    def cell(point):
        return tuple(int(math.floor((point[k]-grid.origin[k])/grid.resolution)) for k in (0, 1))
    if not _supported(grid, start, safe) or not _supported(grid, goal, safe):
        return None
    source, target = cell(start), cell(goal)
    queue, cost, previous = [(0., source)], {source: 0.}, {}
    height, width = len(safe), len(safe[0])
    while queue:
        _, current = heapq.heappop(queue)
        if current == target:
            cells = [target]
            while cells[-1] != source:
                cells.append(previous[cells[-1]])
            cells.reverse()
            return [list(start)]+[[grid.origin[0]+(x+.5)*grid.resolution, grid.origin[1]+(y+.5)*grid.resolution] for x, y in cells]+[list(goal)]
        x, y = current
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nx, ny = x+dx, y+dy
            if not (0 <= nx < width and 0 <= ny < height and safe[ny][nx]):
                continue
            if dx and dy and not (safe[y][nx] and safe[ny][x]):
                continue
            new = cost[current]+math.hypot(dx, dy)
            if new < cost.get((nx, ny), math.inf):
                cost[(nx, ny)], previous[(nx, ny)] = new, current
                heapq.heappush(queue, (new+math.hypot(nx-target[0], ny-target[1]), (nx, ny)))
    return None


def _profile_candidate(mask, top, xx, yy, pose, floor_coef, resolution, policy, preferred_axis=None):
    xy = np.column_stack([xx[mask], yy[mask]])
    z = top[mask]
    center = xy.mean(axis=0)
    _, _, vectors = np.linalg.svd(xy-center, full_matrices=False)
    axis = vectors[0]
    if np.dot(axis, preferred_axis if preferred_axis is not None else center-[pose.x, pose.y]) < 0:
        axis = -axis
    across = np.array([-axis[1], axis[0]])
    u, v = (xy-center)@axis, (xy-center)@across
    length, full_width = np.ptp(u)+resolution, np.ptp(v)+resolution
    base = float(np.r_[center, 1]@floor_coef)
    candidate = {'kind': 'unknown', 'candidate_type': 'raised_surface', 'center': [*center.tolist(), float(np.median(z))],
                 'length_m': float(length), 'width_m': float(full_width), 'height_m': float(np.max(z)-base),
                 'reasons': [], 'strong_geometry': False, 'path': [], 'evidence': []}
    if max(length, full_width) < .5 and candidate['height_m'] >= .12:
        candidate['candidate_type'] = 'compact_obstacle'
        return candidate
    if length < .5 or full_width < .18:
        candidate['reasons'].append('Недостаточный протяжённый участок')
        return candidate
    bins = np.rint((u-u.min())/resolution).astype(int)
    prof, widths, counts = [], [], []
    for index in range(bins.max()+1):
        selected = bins == index
        prof.append(float(np.median(z[selected])) if selected.any() else np.nan)
        # Ширина считается по непрерывному поперечному ряду, содержащему ось.
        # Две полосы по краям провала не образуют сплошной настил.
        lateral = np.sort(v[selected])
        spans = np.split(lateral, np.flatnonzero(np.diff(lateral) > resolution*1.6)+1) if len(lateral) else []
        central = [span for span in spans if span[0]-resolution*.55 <= 0 <= span[-1]+resolution*.55]
        widths.append(float(max((np.ptp(span)+resolution for span in central), default=0.)))
        counts.append(int(selected.sum()))
    prof = np.asarray(prof)
    valid = np.isfinite(prof)
    candidate['max_gap_m'] = float((max((len(run) for run in ''.join('1' if not item else '0' for item in valid).split('0')), default=0))*resolution)
    if not valid.all():
        candidate['reasons'].append('В продольном профиле есть ненаблюдаемый разрыв')
        return candidate
    # Один медианный профиль недостаточен: середина пирамиды или неровного
    # предмета тоже может походить на скат. Проверяем всю поперечную опору.
    lateral_height_error = np.abs(z-prof[bins]-v*np.dot(floor_coef[:2], across))
    candidate['max_cross_surface_error_m'] = float(np.max(lateral_height_error))
    if candidate['max_cross_surface_error_m'] > max(.015, resolution*.65):
        candidate['reasons'].append('Поперечная поверхность не подтверждена как ровная опора')
    # Только самые крайние растровые ряды могут быть неполными.
    width = float(min(widths[1:-1] or widths))
    candidate['width_m'] = width
    smooth = ndimage.median_filter(prof, size=3, mode='nearest')
    peak = int(np.argmax(smooth))
    up_fit = down_fit = None
    if 3 <= peak < len(prof)-3:
        a = np.polyfit(np.arange(peak+1)*resolution, smooth[:peak+1], 1)
        b = np.polyfit(np.arange(len(prof)-peak)*resolution, smooth[peak:], 1)
        up_fit = float(np.sqrt(np.mean((np.polyval(a, np.arange(peak+1)*resolution)-smooth[:peak+1])**2)))
        down_fit = float(np.sqrt(np.mean((np.polyval(b, np.arange(len(prof)-peak)*resolution)-smooth[peak:])**2)))
        if a[0] > .14 and b[0] < -.14 and up_fit < resolution*.65 and down_fit < resolution*.65:
            candidate.update(kind='aframe', candidate_type='observed_ascent_and_descent',
                             max_slope_deg=float(np.degrees(np.arctan(max(abs(a[0]), abs(b[0]))))), max_step_m=0.)
            candidate['evidence'] += ['Наблюдаемы две протяжённые плоскости подъёма и спуска', 'Продольный профиль получен из текущих точек']
    # Плато выделяются по постоянному уровню, а не путём округления любого ската.
    runs = []
    start = 0
    for index in range(1, len(prof)+1):
        if index == len(prof) or abs(prof[index]-np.median(prof[start:index])) > max(.02, resolution*.5):
            if (index-start)*resolution >= .20:
                runs.append((start, index, float(np.median(prof[start:index]))))
            start = index
    levels = [base]+[run[2] for run in runs]+[base]
    rises = np.diff(levels)
    if len(runs) >= 2 and np.sum(rises > max(.035, resolution*.7)) >= 2:
        candidate.update(kind='platforms', candidate_type='two_observed_steps', max_step_m=float(np.max(abs(rises))), max_slope_deg=0.)
        candidate['evidence'] += ['Выделены не менее двух наблюдённых устойчивых уровней и двух подъёмов']
    # Направление платформ задаёт наблюдённый порядок: две ступени вверх,
    # затем протяжённый наклонный спуск. Положение робота его не меняет.
    directions = []
    for reverse in (False, True):
        p = prof[::-1] if reverse else prof
        flat_runs, start = [], 0
        for index in range(1, len(p)+1):
            if index == len(p) or abs(p[index]-p[index-1]) > .006 or np.ptp(p[start:index+1]) > .012:
                if (index-start)*resolution >= .20:
                    flat_runs.append((start, index, float(np.median(p[start:index]))))
                start = index
        if len(flat_runs) < 2:
            continue
        lower_run, upper_run = flat_runs[:2]
        start_low, end_low, low_height = lower_run
        start_high, end_high, high_height = upper_run
        descent = p[end_high-1:]
        if start_low > 1 or start_high-end_low > 1 or low_height-base <= .035 or high_height-low_height <= .035 or len(descent)*resolution < .30:
            continue
        fit = np.polyfit(np.arange(len(descent))*resolution, descent, 1)
        residual = np.max(abs(np.polyval(fit, np.arange(len(descent))*resolution)-descent))
        if fit[0] < -.06 and residual < resolution*.5 and np.max(np.diff(descent)) <= .012:
            directions.append((reverse, max(low_height-base, high_height-low_height), abs(float(fit[0]))))
    if len(directions) == 1:
        reverse, step, grade = directions[0]
        candidate.update(kind='platforms', candidate_type='two_steps_then_observed_descent', max_step_m=float(step),
                         max_slope_deg=float(math.degrees(math.atan(grade))))
        candidate['evidence'].append('Направление: нижняя площадка, верхняя площадка, наблюдённый наклонный спуск')
        if reverse:
            axis, across, u, v, prof = -axis, -across, -u, -v, prof[::-1]
    elif candidate['kind'] == 'platforms':
        candidate['reasons'].append('Не установлен однозначный порядок двух подъёмов и наклонного спуска')
    if candidate['kind'] == 'unknown':
        candidate['candidate_type'] = 'bridge_or_teeter_or_platform'
        candidate['reasons'].append('По верхней поверхности не установлены опоры, шарнир и подвижность настила')
    lower = u.min()
    # Проход по поверхности хранится отдельно от сетки подхода.
    path = []
    for index in range(0, len(prof), max(1, int(round(.10/resolution)))):
        point = center+axis*(lower+index*resolution)
        path.append([float(point[0]), float(point[1]), float(prof[index])])
    if len(prof) > 1:
        point = center+axis*(lower+(len(prof)-1)*resolution)
        path.append([float(point[0]), float(point[1]), float(prof[-1])])
    margin = policy.robot_length/2+policy.clearance+.20
    entry, exit = xy.mean(axis=0)+axis*(u.min()-margin), xy.mean(axis=0)+axis*(u.max()+margin)
    candidate['path'] = [[*entry.tolist(), float(np.r_[entry, 1]@floor_coef)]]+path+[[*exit.tolist(), float(np.r_[exit, 1]@floor_coef)]]
    candidate['max_lip_m'] = float(max(abs(prof[0]-base), abs(prof[-1]-base))) if candidate['kind'] != 'platforms' else 0.
    if candidate['kind'] == 'aframe' or (candidate['kind'] == 'platforms' and len(directions) == 1):
        # Проверяем реальные соседние отсчёты до уровня пола: экстраполяция
        # плоскости сама по себе не является наблюдением низкого края.
        lips = []
        transition_ok = True
        raster_origin = np.array([xx[0, 0], yy[0, 0]])
        transitions = ((u.min(), -1., abs(a[0])), (u.max(), 1., abs(b[0]))) if candidate['kind'] == 'aframe' else ((u.max(), 1., directions[0][2]),)
        for edge, direction, grade in transitions:
            heights = []
            reached_floor = False
            for offset in range(10):
                point = center+axis*(edge+direction*offset*resolution)
                ix, iy = np.rint((point-raster_origin)/resolution).astype(int)
                if not (0 <= iy < len(top) and 0 <= ix < len(top[0])) or not math.isfinite(top[iy, ix]):
                    break
                local_floor = float(np.r_[point, 1]@floor_coef)
                heights.append(float(top[iy, ix]-local_floor))
                if abs(heights[-1]) <= .005:
                    reached_floor = True
                    break
            transition_ok &= reached_floor
            lips.extend(max(0., abs(one-two)-grade*resolution) for one, two in zip(heights, heights[1:]))
        candidate['max_lip_m'] = float(max(lips, default=math.inf))
        if not transition_ok:
            candidate['reasons'].append('Низкие края ската до наблюдённого пола не прослежены')
    if width < policy.robot_width+2*policy.clearance:
        candidate['reasons'].append('Наблюдённая ширина недостаточна для корпуса и зазоров')
    candidate['strong_geometry'] = candidate['kind'] != 'unknown' and not candidate['reasons']
    candidate['axis_xy'] = axis.tolist()
    candidate['bidirectional'] = candidate['kind'] == 'aframe'
    return candidate


class PerceptionPipeline:
    def __init__(self):
        from .cloud_memory import CloudMemory
        self._cloud_memory = CloudMemory()
        self.candidates = []
        self._tracks = []
        self._epoch = None
        self._next = 1
        self._previous_pose = None
        self._previous_t = None

    def _track(self, candidate, t, epoch, policy):
        center = np.asarray(candidate['center'])
        eligible = [track for track in self._tracks if track['kind'] == candidate['kind'] and -1e-6 <= t-track['t'] <= policy.max_candidate_age
                    and np.linalg.norm(center[:2]-track['center'][:2]) < .25]
        track = min(eligible, key=lambda item: np.linalg.norm(center[:2]-item['center'][:2])) if eligible else None
        if track is None:
            track = {'id': f'e{epoch}:surface{self._next}', 'kind': candidate['kind'], 'center': center, 't': t, 'n': int(candidate['strong_geometry']),
                     'axis_xy': candidate.get('axis_xy'), 'first_side': candidate.get('first_side'),
                     'entry_mode': candidate.get('entry_mode')}
            self._next += 1
            self._tracks.append(track)
        elif t > track['t']+1e-6:
            track.update(center=center, t=t, n=track['n']+1 if candidate['strong_geometry'] else 0)
        return track

    def _stable_axis(self, center, kind=None):
        tracks = [track for track in self._tracks if track.get('axis_xy') is not None
                  and (kind is None or track['kind'] == kind) and np.linalg.norm(np.asarray(center)-track['center'][:2]) < .25]
        if not tracks:
            return None
        return min(tracks, key=lambda track: np.linalg.norm(np.asarray(center)-track['center'][:2]))['axis_xy']

    def build_observation(self, t, pose, points, frame_epoch=0, geometry_contract=None, image=None, policy=None):
        policy = policy or Policy()
        if isinstance(pose, dict):
            try:
                pose = Pose(**pose)
            except (TypeError, ValueError):
                pose = None
        if self._epoch != frame_epoch:
            self._tracks, self._epoch = [], frame_epoch
            self._previous_t, self._previous_pose = None, None
        self.candidates = camera_candidates(image)
        try:
            stamp = float(t)
        except (ValueError, TypeError):
            stamp = math.nan
        observation = Observation(stamp, pose, None, frame_epoch=frame_epoch)
        c = geometry_contract if isinstance(geometry_contract, dict) else {}
        errors = _contract_errors(c, pose, frame_epoch, policy)
        if not math.isfinite(stamp) or isinstance(t, bool):
            errors.append('Неконечное время наблюдения')
        if self._previous_t is not None and stamp < self._previous_t:
            errors.append('Время движется назад без новой эпохи координат')
        if errors:
            self._cloud_memory.clear()
            observation.diagnostics = errors+['Пиксельный bbox не превращается в метрическую цель']
            return observation
        t = stamp
        acquired_at = t-float(c['acquisition_age_s'])
        try:
            cloud = np.asarray(points, dtype=float)
            if cloud.ndim != 2 or cloud.shape[1] != 3 or len(cloud) < 30 or not np.isfinite(cloud).all():
                raise ValueError('Нужны не менее 30 конечных метрических точек Nx3')
            resolution = float(c.get('surface_resolution_m', .05))
            if not .02 <= resolution <= .10:
                raise ValueError('Размер наблюдаемой сетки должен быть явно пригоден для локального анализа')
            if c.get('source_kind') == 'occupied_voxel_points':
                size = float(c['voxel_size_m'])
                if resolution+1e-9 < size:
                    raise ValueError('Сетка поверхности не может быть мельче исходного занятого вокселя')
                cloud = cloud+size*np.array([.5, .5, 1.] if c['voxel_reference'] == 'lower_corner' else [0., 0., .5])
            window = c.get('temporal_window_s', 0.)
            if window:
                cloud = self._cloud_memory.merge(cloud, acquired_at, t, float(window),
                    policy.max_observation_age, (frame_epoch, c.get('point_frame'), resolution,
                    c.get('source_kind'), c.get('voxel_size_m'), c.get('voxel_reference')))
                observation.diagnostics.append('Короткая память облака: сохраняются только недавно измеренные точки; занятость не удаляется')
            else:
                self._cloud_memory.clear()
            top, origin, xx, yy, point_cells = _raster(cloud, resolution)
            floor = _floor(top, xx, yy, pose, resolution)
            if floor is None:
                observation.diagnostics.append('Не найдена наблюдённая опорная плоскость возле робота')
                return observation
            coef, floor_height = floor
            grid, raised = _support_grid(top, floor_height, origin, resolution, frame_epoch, policy,
                                        float(c['voxel_size_m']) if c.get('source_kind') == 'occupied_voxel_points' else 0.)
        except (ValueError, TypeError, np.linalg.LinAlgError) as exc:
            # После повреждённого кадра нельзя переносить прежнюю опору
            # через разрыв наблюдений в следующий корректный кадр.
            self._cloud_memory.clear()
            observation.diagnostics.append(str(exc))
            return observation
        observation.grid = grid
        observation.localized = True
        observation.localization_error_m = float(c['localization_error_m'])
        from .navigation import safe_mask
        support_mask = safe_mask(grid, policy, observation.localization_error_m)
        support_height, support_reason = _local_support(top, xx, yy, pose, resolution, policy, c)
        if support_height is not None:
            observation.support_height_m = support_height
            observation.support_verified = True
        elif support_reason:
            observation.diagnostics.append(support_reason)
        labels, number = ndimage.label(raised, np.ones((3, 3)))
        candidates, cones, surface_masks = [], [], {}
        for label_id in range(1, number+1):
            mask = labels == label_id
            if mask.sum() < 6:
                continue
            center_xy = np.array([xx[mask].mean(), yy[mask].mean()])
            item = _profile_candidate(mask, top, xx, yy, pose, coef, resolution, policy, self._stable_axis(center_xy))
            # В маску прохода входит только собственный компонент и наблюдённый
            # низкий край ската; пропущенные измерения не дорисовываются.
            low_edge = ndimage.binary_dilation(mask) & np.isfinite(top) & (top-floor_height <= max(.025, resolution*.6)+1e-8)
            surface_masks[id(item)] = mask | low_edge
            if item['candidate_type'] == 'compact_obstacle':
                selected = mask[point_cells[:, 1], point_cells[:, 0]]
                pts = cloud[selected]
                threshold = np.quantile(pts[:, 2], .65)
                lower = pts[pts[:, 2] <= threshold]
                upper = pts[pts[:, 2] > threshold]
                taper = len(upper) >= 3 and np.max(np.ptp(upper[:, :2], axis=0)) < .8*np.max(np.ptp(lower[:, :2], axis=0))
                if taper and .12 <= item['height_m'] <= 1.0:
                    item['candidate_type'] = 'cone_geometry_candidate'
                    cones.append(item)
                item['reasons'].append('Отдельный компактный предмет не задаёт маршрут слалома')
            candidates.append(item)
        # Название механизма из профиля не является сенсорным свидетельством.
        # Мост и качели остаются ambiguous до отдельного наблюдаемого детектора.
        from .cylinder_slalom import cylinder_rows
        poles, cylinder_candidates = cylinder_rows(cloud, coef, resolution, grid, pose,
                                                   policy, observation.localization_error_m)
        candidates.extend(cylinder_candidates)
        if poles:
            observation.diagnostics.append(f'Цилиндрические стойки: кандидатов {len(poles)}, рядов {len(cylinder_candidates)}; полнота и сопоставление с камерой проверяются отдельно')
        if len(cones) >= 3:
            centers = np.array([item['center'][:2] for item in cones])
            center = centers.mean(axis=0)
            _, _, vectors = np.linalg.svd(centers-center, full_matrices=False)
            axis = vectors[0]
            preferred = self._stable_axis(center, 'slalom')
            if np.dot(axis, preferred if preferred is not None else center-[pose.x, pose.y]) < 0:
                axis = -axis
            across = np.array([-axis[1], axis[0]])
            along = (centers-center)@axis
            order = np.argsort(along)
            spacing = np.diff(along[order])
            radius = math.hypot(policy.robot_length, policy.robot_width)/2+policy.clearance+observation.localization_error_m
            offset = radius+max(max(item['width_m'], item['length_m'])/2 for item in cones)+2*resolution
            from .slalom_route import plan_slalom
            first_side = c.get('slalom_first_side')
            # При продолжении наблюдённого маршрута сохраняем выбранную сторону.
            tracked = [track for track in self._tracks if track['kind']=='slalom'
                       and np.linalg.norm(center-track['center'][:2])<.25]
            if first_side is None and tracked:
                first_side=tracked[0].get('first_side')
            planned=plan_slalom(grid,centers,max(max(item['width_m'],item['length_m'])/2 for item in cones),
                                pose,policy,observation.localization_error_m,first_side,axis,
                                tracked[0].get('entry_mode') if tracked else None)
            routed=planned['path']
            waypoints=planned.get('alternating_waypoints_xy',[])
            strong = bool(np.all(spacing > .65) and np.max(abs((centers-center)@across)) < .25 and routed)
            candidates.append({'kind': 'slalom', 'candidate_type': 'alternating_cones', 'center': [*center.tolist(), float(np.r_[center, 1]@coef)],
                               'width_m': float(2*offset), 'strong_geometry': strong, 'max_slope_deg': 0., 'max_step_m': 0., 'max_gap_m': 0., 'max_lip_m': 0.,
                               'path': [[*list(point), float(np.r_[point, 1]@coef)] for point in routed],
                               'alternating_waypoints_xy': waypoints, 'first_side': planned.get('first_side'),
                               'entry_mode': planned.get('entry_mode'),
                               'axis_xy': axis.tolist(), 'bidirectional': False,
                               'reasons': [] if strong else ['Не подтверждены расстояния, опора всей траектории или разрешённая первая сторона слалома'],
                               'evidence': ['Не менее трёх сужающихся вверх геометрических компонентов', 'Боковые точки маршрута чередуются относительно оси конусов']})
        for item in candidates:
            if item.get('path') and item['kind'] != 'slalom':
                endpoints_ok = _supported(grid, item['path'][0][:2], support_mask) and _supported(grid, item['path'][-1][:2], support_mask)
                if not endpoints_ok:
                    item['strong_geometry'] = False
                    item['reasons'].append('Полная опора входа или выхода не подтверждена')
            track = self._track(item, acquired_at, frame_epoch, policy)
            verified = bool(item['strong_geometry'] and track['n'] >= 3)
            item.update(id=track['id'], confirmations=track['n'], geometry_verified=verified,
                        confidence=.9 if item['strong_geometry'] else .35, frame_epoch=frame_epoch)
            dx, dy = item['center'][0]-pose.x, item['center'][1]-pose.y
            item['relative_body_xy'] = [math.cos(pose.yaw)*dx+math.sin(pose.yaw)*dy,
                                       -math.sin(pose.yaw)*dx+math.cos(pose.yaw)*dy]
            if item['kind'] in ('aframe', 'platforms', 'bridge', 'teeter', 'slalom') and item['path']:
                observation.obstacles.append(Obstacle(item['id'], item['kind'], tuple(tuple(p) for p in item['path']),
                    item['width_m'], acquired_at, item['confidence'], track['n'], verified, verified, item.get('bidirectional', False),
                    item.get('max_slope_deg', 90.), item.get('max_step_m', 0.), item.get('max_gap_m', 0.), item.get('max_lip_m', 0.),
                    frame_epoch, tuple(item['evidence']+item['reasons'])))
                if verified:
                    surface_cells = np.array(grid.cells, dtype=np.int8)
                    own_mask = surface_masks.get(id(item))
                    if own_mask is not None:
                        surface_cells[own_mask & (surface_cells == 1)] = 0
                    observation.traversal_grids[item['id']] = Grid(grid.origin, grid.resolution, surface_cells.tolist(), frame_epoch, True)
        self.candidates += candidates
        observation.diagnostics += ['Grid0: наблюдённая опорная плоскость; габариты и неизвестность учитываются navigation.safe_mask',
                                    'Поверхности и пиксельные кандидаты камеры не объединены без проверенной калибровки',
                                    'Механическая прочность и сцепление не выводятся из формы облака']
        self._tracks = [track for track in self._tracks if t-track['t'] <= policy.max_candidate_age]
        if self._previous_pose is not None:
            delta = (pose.yaw-self._previous_pose.yaw+math.pi) % (2*math.pi)-math.pi
            observation.diagnostics.append(f'Изменение yaw в переданной системе: {math.degrees(delta):.4f} град; относительные координаты кандидатов рассчитаны без поправки курса')
        self._previous_t, self._previous_pose = stamp, pose
        return observation


_DEFAULT = PerceptionPipeline()


def build_observation(t, pose, points, frame_epoch=0, geometry_contract=None, image=None, policy=None):
    return _DEFAULT.build_observation(t, pose, points, frame_epoch, geometry_contract, image, policy)


def reset_tracking():
    global _DEFAULT
    _DEFAULT = PerceptionPipeline()


def adapter_observe(capture, pipeline=None, policy=None):
    """capture — данные адаптера; профильный контракт передаётся без повышения доверия."""
    engine = pipeline or _DEFAULT
    return engine.build_observation(capture['t'], capture.get('pose'), capture.get('points'), capture.get('frame_epoch', 0),
                                    capture.get('geometry_contract'), capture.get('image'), policy)
