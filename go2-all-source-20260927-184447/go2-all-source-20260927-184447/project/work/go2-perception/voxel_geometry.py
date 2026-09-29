"""Геометрический анализ карты Go2 без соединения с роботом и команд движения.

Массив positions декодера libvoxel — uint8 координаты вершин сетки, а не
байтовое представление float. Преобразование: xyz = origin + resolution * ijk.
Основание: legion1581/unitree_ui/src/ui/scene/voxel-map.ts, строки 101–115,
и unitree_webrtc_connect/lidar/lidar_decoder_libvoxel.py, строки 131–147.
Нативный декодер уже возвращает метрические точки занятых вокселей.

Анализ верхней огибающей не восстанавливает лучи, свободное пространство,
сцепление, прочность поверхности или доступность подхода. Наклонная плоскость
остается кандидатом; этот модуль не разрешает движение.
"""
from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np
from scipy import ndimage


SOURCE_URLS = [
    "https://github.com/legion1581/unitree_ui/blob/main/src/ui/scene/voxel-map.ts",
    "https://github.com/legion1581/unitree_webrtc_connect/blob/master/unitree_webrtc_connect/lidar/lidar_decoder_libvoxel.py",
    "https://github.com/legion1581/unitree_webrtc_connect/blob/master/unitree_webrtc_connect/lidar/lidar_decoder_native.py",
]


def _metadata(value):
    """Принимает data, полное сообщение или строку samples.jsonl."""
    if "message" in value:
        value = value["message"]
    if "resolution" not in value and isinstance(value.get("data"), dict):
        value = value["data"]
    resolution = float(value["resolution"])
    origin = np.asarray(value["origin"], dtype=float)
    width = np.asarray(value["width"], dtype=int)
    if not 0.005 <= resolution <= 0.5:
        raise ValueError("Неподдерживаемый размер вокселя")
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError("origin должен содержать три конечных числа")
    if width.shape != (3,) or np.any(width < 1) or np.any(width > 4096):
        raise ValueError("Некорректный размер карты")
    if value.get("frame_id") != "odom":
        raise ValueError("Поддерживается только карта в системе odom")
    return value, origin, resolution, width


def _pose(value, frame_id="odom"):
    """Поза ROBOTODOM или явный словарь position/yaw/frame_id."""
    if "message" in value:
        value = value["message"]
    if "data" in value:
        value = value["data"]
    supplied_frame = value.get("frame_id", value.get("header", {}).get("frame_id"))
    if supplied_frame != frame_id:
        raise ValueError("Поза и карта должны явно иметь одинаковый frame_id")
    pose = value.get("pose", value)
    position = pose["position"]
    if isinstance(position, dict):
        position = [position[a] for a in "xyz"]
    position = np.asarray(position, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError("Некорректная позиция робота")
    if "yaw" in pose:
        yaw = float(pose["yaw"])
    else:
        q = pose["orientation"]
        q = np.asarray([q[a] for a in ("x", "y", "z", "w")], dtype=float)
        norm = np.linalg.norm(q)
        if not np.isfinite(q).all() or norm < 1e-6:
            raise ValueError("Некорректный кватернион робота")
        x, y, z, w = q / norm
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y*y + z*z))
    if not math.isfinite(yaw):
        raise ValueError("Некорректный курс робота")
    return position, yaw


def load_voxel_points(npz_path, metadata):
    """Возвращает метрические точки, метаданные и описание декодирования.

    Для mesh возвращаются уникальные вершины поверхности. Это НЕ полная
    исходная сетка занятых вокселей, поэтому оценки mesh/native разнятся
    примерно на размер ячейки и не должны объединяться как один тип точек.
    """
    meta, origin, resolution, width = _metadata(metadata)
    with np.load(npz_path, allow_pickle=False) as data:
        point_keys = [key for key in data.files if key == "points" or key.endswith("_points")]
        position_keys = [key for key in data.files if key == "positions" or key.endswith("_positions")]
        if len(point_keys) == 1:
            points = np.asarray(data[point_keys[0]], dtype=float)
            if points.ndim != 2 or points.shape[1] != 3:
                raise ValueError("Ожидается массив нативных точек Nx3")
            encoding = "native_metric_occupied_voxels"
            info = {"encoding": encoding, "formula": "points уже в метрах; повторного преобразования нет"}
        elif len(position_keys) == 1:
            raw = data[position_keys[0]]
            if raw.dtype != np.uint8 or raw.ndim != 1 or len(raw) % 12:
                raise ValueError("Ожидается плоский uint8 positions по 12 координат на грань")
            grid = raw.reshape(-1, 3)
            if len(grid) and np.any(grid > width):
                raise ValueError("Вершины mesh вышли за width карты")
            quads = grid.reshape(-1, 4, 3).astype(np.int16)
            sizes = np.ptp(quads, axis=1)
            if len(sizes) and not np.all(np.sort(sizes, axis=1) == [0, 1, 1]):
                raise ValueError("Геометрия не соответствует единичным квадратным граням libvoxel")
            faces = meta.get("data", {}).get("face_count")
            if faces is not None and len(quads) != int(faces):
                raise ValueError("face_count не соответствует сохранённому NPZ")
            index_keys = [key for key in data.files if key == "indices" or key.endswith("_indices")]
            if len(index_keys) != 1:
                raise ValueError("Для проверки mesh необходимы indices")
            indices = data[index_keys[0]]
            if indices.dtype != np.uint32 or indices.size != len(quads) * 6:
                raise ValueError("Некорректные индексы треугольников mesh")
            if indices.size and int(indices.max()) >= len(grid):
                raise ValueError("Индекс mesh за границами вершин")
            points = np.unique(grid, axis=0).astype(float) * resolution + origin
            encoding = "libvoxel_metric_surface_vertices"
            info = {"encoding": encoding, "formula": "xyz_m = uint8_xyz * resolution + origin",
                    "face_count": len(quads), "vertex_count_with_duplicates": len(grid)}
        else:
            raise ValueError("Не найден однозначный массив points или positions")
    if not len(points) or not np.isfinite(points).all():
        raise ValueError("Карта пуста либо содержит NaN/Inf")
    grid_float = (points - origin) / resolution
    if np.any(grid_float < -1e-4) or np.any(grid_float > width + 1e-4):
        raise ValueError("Метрические точки не согласованы с origin/resolution/width")
    if np.max(np.abs(grid_float - np.rint(grid_float))) > 1e-3:
        raise ValueError("Точки не лежат на указанной воксельной сетке")
    info.update({"point_count": len(points), "sources": SOURCE_URLS,
                 "quantization_m": resolution, "grid_bounds_validated": True})
    return points, meta, info


def _heightmap(points, origin, resolution, width):
    """Верхняя измеренная точка каждого столбца, пустые столбцы остаются NaN."""
    grid = np.rint((points - origin) / resolution).astype(int)
    # У mesh крайняя вершина может иметь индекс width, у нативных вокселей
    # последний индекс width-1. Не добавляем нативной карте пустую рамку.
    shape = np.maximum(width[:2], grid[:, :2].max(axis=0) + 1)
    height = np.full(tuple(shape), -np.inf)
    np.maximum.at(height, (grid[:, 0], grid[:, 1]), points[:, 2])
    height[~np.isfinite(height)] = np.nan
    xx, yy = np.meshgrid(origin[0] + np.arange(height.shape[0]) * resolution,
                         origin[1] + np.arange(height.shape[1]) * resolution, indexing="ij")
    return height, xx, yy


def _local_planes(height, resolution):
    """Локальные плоскости по измеренным клеткам, пропуски не заполняются."""
    radius = max(3, int(round(0.18 / resolution)))
    coords = np.arange(-radius, radius + 1) * resolution
    dx, dy = np.meshgrid(coords, coords, indexing="ij")
    valid = np.isfinite(height).astype(float)
    zz = np.nan_to_num(height)
    one = np.ones_like(dx)
    moments = [ndimage.correlate(valid, kernel, mode="constant", cval=0)
               for kernel in (dx*dx, dx*dy, dx, dy*dy, dy, one)]
    sxx, sxy, sx, syy, sy, count = moments
    matrix = np.empty(height.shape + (3, 3))
    matrix[..., 0, :] = np.stack([sxx, sxy, sx], axis=-1)
    matrix[..., 1, :] = np.stack([sxy, syy, sy], axis=-1)
    matrix[..., 2, :] = np.stack([sx, sy, count], axis=-1)
    rhs = np.stack([ndimage.correlate(zz, kernel, mode="constant", cval=0)
                    for kernel in (dx, dy, one)], axis=-1)
    good = (count >= one.size * 0.8) & (np.linalg.det(matrix) > 1e-8)
    coeff = np.full(height.shape + (3,), np.nan)
    coeff[good] = np.linalg.solve(matrix[good], rhs[good, :, None])[..., 0]
    sum_z2 = ndimage.correlate(zz*zz, one, mode="constant", cval=0)
    variance = (sum_z2 - np.nansum(coeff * rhs, axis=-1)) / np.maximum(count, 1)
    rms = np.sqrt(np.maximum(variance, 0))
    rms[~good] = np.nan
    slope = np.degrees(np.arctan(np.linalg.norm(coeff[..., :2], axis=-1)))
    return coeff, slope, rms, good


def _fit_plane(points, resolution, iterations=4):
    """Устойчивое приближение z=ax+by+c, без допущения о фиксированном поле."""
    matrix = np.column_stack([points[:, :2], np.ones(len(points))])
    keep = np.ones(len(points), dtype=bool)
    for _ in range(iterations):
        if keep.sum() < 6:
            return None
        coeff, _, rank, _ = np.linalg.lstsq(matrix[keep], points[keep, 2], rcond=None)
        if rank < 3:
            return None
        residual = np.abs(matrix @ coeff - points[:, 2])
        med = np.median(residual[keep])
        threshold = max(resolution * 0.70, min(resolution * 1.25, med * 2.5))
        keep = residual <= threshold
    normal = np.array([-coeff[0], -coeff[1], 1.0])
    normal /= np.linalg.norm(normal)
    return coeff, normal, residual, keep


def _floor(height, xx, yy, slope, rms, robot, resolution):
    radial = np.hypot(xx - robot[0], yy - robot[1])
    # Оценка возле робота. Высота тела не используется как измерение пола.
    possible = ((radial > 0.45) & (radial < 1.8) & (height < robot[2] - 0.14)
                & (height > robot[2] - 0.8) & (slope < 7) & (rms < resolution * .8))
    if possible.sum() < 30:
        return None
    heights = height[possible]
    bins = np.rint(heights / resolution).astype(int)
    values, counts = np.unique(bins, return_counts=True)
    mode = values[np.argmax(counts)] * resolution
    seed = possible & (np.abs(height - mode) < 1.6 * resolution)
    points = np.column_stack([xx[seed], yy[seed], height[seed]])
    fit = _fit_plane(points, resolution)
    if fit is None:
        return None
    coeff, normal, residual, keep = fit
    if math.degrees(math.acos(float(normal[2]))) > 7 or keep.sum() < 30:
        return None
    return {"equation_z_abc": coeff.tolist(), "normal_odom": normal.tolist(),
            "height_at_robot_xy_m": float(np.r_[robot[:2], 1.] @ coeff),
            "support_cells": int(keep.sum()), "rms_m": float(np.sqrt(np.mean(residual[keep]**2))),
            "support_radius_from_robot_m": [.45, 1.8],
            "support_bounds_min_m": points[keep].min(axis=0).tolist(),
            "support_bounds_max_m": points[keep].max(axis=0).tolist(),
            "status": "local_floor_estimate"}


def _candidate(component, height, xx, yy, resolution, floor, robot, yaw, index):
    points = np.column_stack([xx[component], yy[component], height[component]])
    if len(points) < 30:
        return None
    fit = _fit_plane(points, resolution)
    if fit is None:
        return None
    coeff, normal, residual, keep = fit
    support = points[keep]
    if len(support) < 30:
        return None
    gradient = coeff[:2]
    slope = math.degrees(math.atan(np.linalg.norm(gradient)))
    if not 8 <= slope <= 55:
        return None
    uphill = gradient / np.linalg.norm(gradient)
    across = np.array([-uphill[1], uphill[0]])
    along = support[:, :2] @ uphill
    cross = support[:, :2] @ across
    lo, hi = np.quantile(along, [.03, .97])
    left, right = np.quantile(cross, [.03, .97])
    length, width = hi - lo + resolution, right - left + resolution
    rise = float(np.quantile(support[:, 2], .97) - np.quantile(support[:, 2], .03))
    area = len(support) * resolution**2
    coverage = min(1., area / max(length * width, resolution**2))
    if length < .35 or width < .30 or rise < .15 or area < .12 or coverage < .4:
        return None
    if np.mean(keep) < .75:
        return None
    floor_coeff = np.asarray(floor["equation_z_abc"]) if floor else None
    bottom_xy = lo * uphill + (left + right) / 2 * across
    bottom_z = float(np.r_[bottom_xy, 1.] @ coeff)
    bottom_above_floor = None if floor_coeff is None else float(bottom_z - np.r_[bottom_xy, 1.] @ floor_coeff)
    floor_extrapolated = bool(np.linalg.norm(bottom_xy-robot[:2]) > 1.8)
    # Подход находится со стороны нижнего края ската. Это геометрическая
    # точка, не проверенный маршрут и не команда движения.
    approach_xy = bottom_xy - .50 * uphill
    approach_z = None if floor_coeff is None else float(np.r_[approach_xy, 1.] @ floor_coeff)
    desired_yaw = math.atan2(uphill[1], uphill[0])
    delta = approach_xy - robot[:2]
    c, s = math.cos(yaw), math.sin(yaw)
    relative_xy = [c*delta[0]+s*delta[1], -s*delta[0]+c*delta[1]]
    polygon = []
    for u, v in ((lo, left), (hi, left), (hi, right), (lo, right)):
        xy = u * uphill + v * across
        polygon.append([float(xy[0]), float(xy[1]), float(np.r_[xy, 1.] @ coeff)])
    reasons = ["Не проверены свободный путь к подходу, сцепление и прочность поверхности",
               "Нужны повторное наблюдение с другой позиции и согласование с камерой"]
    if bottom_above_floor is None or abs(bottom_above_floor) > .15:
        reasons.append("Нижний край не согласован с оценённым полом; возможны ступень, край предмета или неполный обзор")
    if floor_extrapolated:
        reasons.append("Сравнение нижнего края с полом экстраполирует локальную плоскость за область её оценки")
    return {"id": f"incline_{index}", "classification": "inclined_plane_candidate",
            "status": "geometry_only_unverified", "slope_deg": float(slope),
            "normal_odom": normal.tolist(), "equation_z_abc": coeff.tolist(),
            "centroid_odom_m": np.mean(support, axis=0).tolist(),
            "polygon_odom_m": polygon, "observed_length_m": float(length),
            "observed_width_m": float(width), "observed_rise_m": rise,
            "support_cells": len(support), "observed_area_m2": float(area),
            "coverage_fraction": float(coverage),
            "rms_m": float(np.sqrt(np.mean(residual[keep]**2))),
            "p95_residual_m": float(np.quantile(residual[keep], .95)),
            "lower_edge_above_floor_m": bottom_above_floor,
            "local_floor_extrapolated_at_lower_edge": floor_extrapolated,
            "slope_relative_local_floor_deg": None if floor is None else math.degrees(math.acos(float(np.clip(np.dot(normal, floor["normal_odom"]), -1, 1)))),
            "uphill_direction_xy": uphill.tolist(),
            "approach": {"position_odom_m": [float(approach_xy[0]), float(approach_xy[1]), approach_z],
                         "target_yaw_rad": desired_yaw, "position_robot_xy_m": relative_xy,
                         "distance_xy_m": float(np.linalg.norm(delta)),
                         "yaw_error_rad": float(math.atan2(math.sin(desired_yaw-yaw), math.cos(desired_yaw-yaw))),
                         "clearance_verified": False, "motion_authorized": False},
            "limitations": reasons}


def analyze_points(points, metadata, robot_pose, decode_info=None):
    """Вычисляет геометрию; пригодно также для проверки на синтетических данных."""
    meta, origin, resolution, width = _metadata(metadata)
    robot, yaw = _pose(robot_pose, meta["frame_id"])
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError("Нужны непустые конечные метрические точки Nx3")
    grid_float = (points-origin)/resolution
    if np.any(grid_float < -1e-4) or np.any(grid_float > width+1e-4):
        raise ValueError("Точки за пределами карты")
    height, xx, yy = _heightmap(points, origin, resolution, width)
    coeff, slope, rms, good = _local_planes(height, resolution)
    floor = _floor(height, xx, yy, slope, rms, robot, resolution)
    # Ближний корпус и область под ним не участвуют в классификации.
    away = np.hypot(xx-robot[0], yy-robot[1]) > .45
    inclined = (good & away & np.isfinite(height) & (slope >= 8) & (slope <= 55)
                & (rms <= resolution * .65))
    # Направление локального уклона раздельно по 30-градусным секторам:
    # перпендикулярные боковые грани не должны образовывать один скат.
    angles = np.arctan2(coeff[..., 1], coeff[..., 0])
    candidates = []
    for sector in range(12):
        center = -math.pi + sector * math.pi / 6
        diff = np.arctan2(np.sin(angles-center), np.cos(angles-center))
        eligible = inclined & (np.abs(diff) < math.pi / 6)
        labels, nlabels = ndimage.label(eligible, np.ones((3, 3)))
        counts = np.bincount(labels.ravel())
        for label_id in np.flatnonzero(counts[1:] >= 30) + 1:
            item = _candidate(labels == label_id, height, xx, yy, resolution,
                              floor, robot, yaw, len(candidates)+1)
            if item is None:
                continue
            # Один наклон попадает в соседние угловые сектора.
            duplicate = None
            for old in candidates:
                dist = np.linalg.norm(np.asarray(item["centroid_odom_m"])-old["centroid_odom_m"])
                dot = np.dot(item["normal_odom"], old["normal_odom"])
                if dist < .45 and dot > .97:
                    duplicate = old
                    break
            if duplicate is None:
                candidates.append(item)
            elif item["support_cells"] > duplicate["support_cells"]:
                candidates[candidates.index(duplicate)] = item
    candidates.sort(key=lambda item: (-item["support_cells"], item["rms_m"]))
    for index, item in enumerate(candidates, 1):
        item["id"] = f"incline_{index}"
    # Высокий скачок верхней огибающей — ребро препятствия, а не пандус.
    jump = np.zeros_like(height, dtype=bool)
    for axis in (0, 1):
        step = np.abs(np.diff(height, axis=axis)) > max(.12, 2.4*resolution)
        if axis == 0:
            jump[:-1] |= step
            jump[1:] |= step
        else:
            jump[:, :-1] |= step
            jump[:, 1:] |= step
    horizontal = good & away & (slope < 7) & (rms < resolution*.8)
    elevated = np.zeros_like(horizontal)
    if floor:
        a, b, c = floor["equation_z_abc"]
        elevated = horizontal & (height - (a*xx+b*yy+c) > 1.5*resolution)
    labels, _ = ndimage.label(elevated, np.ones((3, 3)))
    horizontal_patches = []
    counts = np.bincount(labels.ravel())
    for label_id in np.flatnonzero(counts[1:] >= 24)+1:
        select = labels == label_id
        horizontal_patches.append({"classification": "raised_horizontal_surface",
                                   "centroid_odom_m": [float(xx[select].mean()), float(yy[select].mean()), float(height[select].mean())],
                                   "observed_area_m2": float(select.sum()*resolution**2),
                                   "semantic_identity": "unknown: мат, площадка, коробка и т.п. по одному лидару не различимы"})
    return {"schema_version": 1, "frame_id": meta["frame_id"], "resolution_m": resolution,
            "source_kind": "occupied_voxel_map_not_raw_lidar_rays", "decode": decode_info or {},
            "robot_pose": {"position_odom_m": robot.tolist(), "yaw_rad": yaw},
            "map": {"point_count": len(points), "observed_xy_cells": int(np.isfinite(height).sum()),
                    "bounds_min_m": points.min(axis=0).tolist(), "bounds_max_m": points.max(axis=0).tolist(),
                    "height_quantiles_m": np.quantile(height[np.isfinite(height)], [.05, .5, .95]).tolist(),
                    "unknown_xy_fraction": float(np.mean(~np.isfinite(height))),
                    "vertical_step_cells": int(jump.sum())},
            "floor": floor, "inclined_planes": candidates,
            "raised_horizontal_surfaces": horizontal_patches,
            "motion_authorized": False,
            "limitations": ["Это занятые воксели накопленной карты; отсутствие точки не подтверждает свободное место",
                            "Верхняя огибающая теряет нависающие поверхности и нижние слои",
                            "Оценка пола локальная; её нельзя продолжать на всё помещение как доказанный пол",
                            "Точность ограничена размером ячейки и качеством исходной одометрии",
                            "Метки времени карты и позы должны согласовываться внешним сборщиком",
                            "Кандидаты не подтверждают способность робота подняться или наличие свободного подхода"]}


def analyze_voxels(npz_path, metadata, robot_pose):
    """Публичный интерфейс: полностью JSON-совместимый отчёт по одному NPZ."""
    points, meta, info = load_voxel_points(npz_path, metadata)
    return analyze_points(points, meta, robot_pose, info)


def self_test():
    """Проверки единиц, направления, ложных скатов и отказов на плохом входе."""
    import tempfile
    import time
    metadata = {"origin": [-3., -3., -.5], "resolution": .05,
                "width": [128, 128, 48], "frame_id": "odom"}
    robot = {"frame_id": "odom", "position": [0, 0, .32], "yaw": .2}
    xx, yy = np.meshgrid(np.arange(-3, 3.4, .05), np.arange(-3, 3.4, .05), indexing="ij")
    reports = []
    rng = np.random.default_rng(4)
    for name, angle in (("flat", None), ("box", None), ("ramp_0", 0),
                        ("ramp_60", 60), ("ramp_135", 135), ("ramp_noisy", 60),
                        ("sparse_unknown", 60)):
        zz = np.zeros_like(xx)
        if name == "box":
            zz[(xx > .8) & (xx < 2.2) & (abs(yy) < .6)] = .4
        if angle is not None:
            a = math.radians(angle)
            u, v = xx*math.cos(a) + yy*math.sin(a), -xx*math.sin(a) + yy*math.cos(a)
            mask = (u > .8) & (u < 2.2) & (abs(v) < .6)
            zz[mask] = (u[mask]-.8) * .35
        if name == "ramp_noisy":
            zz += rng.normal(0, .012, zz.shape)
        zz = np.round(zz/.05)*.05
        points = np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])
        if name == "sparse_unknown":
            points = points[rng.random(len(points)) < .15]
        started = time.perf_counter()
        out = analyze_points(points, metadata, robot)
        elapsed = time.perf_counter()-started
        found = out["inclined_planes"]
        if name in ("flat", "box", "sparse_unknown"):
            assert not found, f"Ложный скат: {name}"
        else:
            assert found, f"Скат не обнаружен: {name}"
            best = max(found, key=lambda item: item["support_cells"])
            assert abs(best["slope_deg"]-math.degrees(math.atan(.35))) < 3
            assert np.dot(best["uphill_direction_xy"], [math.cos(a), math.sin(a)]) > .98
            assert not best["approach"]["motion_authorized"]
        json.dumps(out, allow_nan=False)
        reports.append({"case": name, "candidates": len(found),
                        "slopes": [item["slope_deg"] for item in found], "seconds": elapsed})
    with tempfile.TemporaryDirectory(prefix="voxel-test-") as directory:
        path = Path(directory)/"mesh.npz"
        grid = np.array([[2, 3, 4], [3, 3, 4], [2, 4, 4], [3, 4, 4]], dtype=np.uint8)
        indices = np.array([0, 1, 2, 1, 2, 3], dtype=np.uint32)
        np.savez(path, positions=grid.ravel(), indices=indices)
        actual, _, _ = load_voxel_points(path, metadata)
        assert np.allclose(actual, np.unique(grid, axis=0)*.05 + [-3, -3, -.5])
        native_path = Path(directory)/"native.npz"
        np.savez(native_path, points=actual)
        recovered, _, _ = load_voxel_points(native_path, metadata)
        assert np.allclose(actual, recovered), "Нативные точки повторно масштабированы"
        reports.append({"case": "mesh_and_native_units", "passed": True})
        indices[-1] = 100
        np.savez(path, positions=grid.ravel(), indices=indices)
        try:
            load_voxel_points(path, metadata)
        except ValueError:
            reports.append({"case": "invalid_mesh_rejected", "passed": True})
        else:
            raise AssertionError("Повреждённый индекс mesh принят")
    try:
        analyze_points(points, metadata, dict(robot, frame_id="body"))
    except ValueError:
        reports.append({"case": "frame_mismatch_rejected", "passed": True})
    else:
        raise AssertionError("Смешаны системы odom и body")
    return {"passed": True, "checks": reports}


def save_overlay(npz_path, metadata, robot_pose, analysis, output_path):
    """Стандартный график: план карты и трёхмерная верхняя огибающая."""
    import os
    import tempfile
    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "go2-perception-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    points, meta, _ = load_voxel_points(npz_path, metadata)
    _, origin, resolution, width = _metadata(meta)
    height, xx, yy = _heightmap(points, origin, resolution, width)
    robot, yaw = _pose(robot_pose)
    observed = np.isfinite(height)
    fig = plt.figure(figsize=(16, 7), facecolor="#f7f8fa")
    ax = fig.add_subplot(121)
    sc = ax.scatter(xx[observed], yy[observed], c=height[observed], s=4, cmap="viridis", vmin=-.25, vmax=.8)
    ax.scatter(robot[0], robot[1], marker="o", s=65, color="red", edgecolor="white", zorder=5)
    ax.arrow(robot[0], robot[1], .42*math.cos(yaw), .42*math.sin(yaw), width=.025, color="red")
    ax.set(xlabel="X odom, м", ylabel="Y odom, м", title="Измеренная верхняя огибающая; красный — робот")
    ax.set_aspect("equal")
    ax.grid(alpha=.2)
    ax3 = fig.add_subplot(122, projection="3d")
    stride = max(1, int(observed.sum()/10000))
    ax3.scatter(xx[observed][::stride], yy[observed][::stride], height[observed][::stride],
                c=height[observed][::stride], s=2, cmap="viridis", vmin=-.25, vmax=.8)
    for item in analysis["inclined_planes"]:
        poly = np.asarray(item["polygon_odom_m"])
        closed = np.vstack([poly, poly[0]])
        ax.plot(closed[:, 0], closed[:, 1], color="#ff6a00", linewidth=2)
        center = np.asarray(item["centroid_odom_m"])
        ax.text(center[0], center[1], f"{item['id']}: {item['slope_deg']:.0f}°", fontsize=8, color="#b13e00",
                bbox={"facecolor": "white", "alpha": .8, "edgecolor": "none"})
        approach = item["approach"]["position_odom_m"]
        ax.scatter(*approach[:2], marker="x", color="#c33baa", s=55)
        ax3.add_collection3d(Poly3DCollection([poly], alpha=.5, facecolor="#ff6a00"))
    ax3.set(xlabel="X odom, м", ylabel="Y odom, м", zlabel="Z odom, м", title="Оранжевый — наклонные кандидаты")
    ax3.view_init(elev=29, azim=-65)
    ax3.set_box_aspect((1, 1, .42))
    cax = fig.add_axes([.946, .24, .013, .51])
    fig.colorbar(sc, cax=cax, label="Высота Z, м")
    fig.suptitle(f"Go2 | {Path(npz_path).name} | ячейка {resolution*100:.0f} см | подходы не проверены", fontsize=15)
    fig.subplots_adjust(left=.055, right=.89, top=.87, bottom=.10, wspace=.20)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def _array_specs(value, prefix="data"):
    """Те же имена NPZ, что у рекурсивного сохранения массива sensor_probe."""
    found = {}
    if isinstance(value, dict):
        if "array_shape" in value and "dtype" in value:
            found[prefix] = (tuple(value["array_shape"]), str(value["dtype"]))
        else:
            for key, child in value.items():
                found.update(_array_specs(child, prefix + "_" + str(key)))
    return found


def load_capture_pairs(log_dir, allow_legacy_order=False):
    """Проверяет полный набор и возвращает пары file/metadata/pose/pairing.

    Новые записи требуют явного npz_file в samples.jsonl. Для старых полных
    записей порядок допускается только по явному флагу; размеры и типы
    массивов проверяются, но не являются уникальным идентификатором кадра.
    """
    log_dir = Path(log_dir)
    summary = json.loads((log_dir/"summary.json").read_text(encoding="utf-8"))
    expected = summary.get("lidar_files")
    if not isinstance(expected, list) or not expected:
        raise ValueError("summary.lidar_files должен содержать непустой список ожидаемых NPZ")
    if any(not isinstance(name, str) or not re.fullmatch(r"lidar_[0-9]+\.npz", name) for name in expected):
        raise ValueError("Некорректное имя NPZ в summary.lidar_files")
    if len(set(expected)) != len(expected):
        raise ValueError("Повторяющиеся имена в summary.lidar_files")
    actual = {path.name for path in log_dir.glob("lidar_*.npz") if path.is_file()}
    missing, unexpected = set(expected)-actual, actual-set(expected)
    if missing or unexpected:
        raise ValueError(f"Неполный или несогласованный набор NPZ: отсутствуют {sorted(missing)}, лишние {sorted(unexpected)}")
    records = [json.loads(line) for line in (log_dir/"samples.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
    lidars = [record for record in records if record.get("topic") == "ULIDAR_ARRAY"]
    poses = [record for record in records if record.get("topic") == "ROBOTODOM"]
    if not poses or any(not math.isfinite(float(pose["elapsed_s"])) for pose in poses):
        raise ValueError("Нет сохранённых поз с конечным временем приёма")
    explicit = [record for record in lidars if record.get("npz_file")]
    if explicit:
        names = [record["npz_file"] for record in explicit]
        if any(not isinstance(name, str) for name in names) or len(set(names)) != len(names) or set(names) != set(expected):
            raise ValueError("Явные npz_file должны однозначно покрывать полный список summary.lidar_files")
        by_name = {record["npz_file"]: record for record in explicit}
        pairing = "explicit_npz_file_and_verified_array_shapes"
    else:
        if not allow_legacy_order:
            raise ValueError("У старой записи нет npz_file; для полной архивной серии явно укажите --allow-legacy-order")
        if [int(name[6:-4]) for name in expected] != list(range(len(expected))):
            raise ValueError("Legacy допускается только для последовательной серии lidar_00… без пропусков")
        array_records = [record for record in lidars if _array_specs(record.get("message", {}))]
        if len(array_records) < len(expected):
            raise ValueError("Недостаточно сообщений лидара с array_shape для полной серии")
        by_name = {name: array_records[int(name[6:-4])] for name in expected}
        pairing = "legacy_order_explicitly_allowed_shapes_verified_identity_not_proven"
    pairs = []
    for name in expected:
        record = by_name[name]
        if not math.isfinite(float(record["elapsed_s"])):
            raise ValueError(f"{name}: некорректное время приёма лидара")
        specs = _array_specs(record.get("message", {}))
        if not specs:
            raise ValueError(f"{name}: сообщение не содержит array_shape и dtype")
        path = log_dir/name
        with np.load(path, allow_pickle=False) as data:
            if set(data.files) != set(specs):
                raise ValueError(f"{name}: состав массивов NPZ не соответствует сообщению")
            for key, (shape, dtype) in specs.items():
                if data[key].shape != shape or str(data[key].dtype) != dtype:
                    raise ValueError(f"{name}: размер или тип массива {key} не соответствует сообщению")
        pose = min(poses, key=lambda item: abs(item["elapsed_s"]-record["elapsed_s"]))
        pairs.append({"file": path, "metadata": record, "pose": pose, "pairing": pairing})
    return pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_dir", type=Path, nargs="?")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/go2-perception"))
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--allow-legacy-order", action="store_true",
                        help="Явно разрешить порядок старой полной серии без поля npz_file")
    args = parser.parse_args()
    if args.self_test:
        report = self_test()
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir/"voxel-synthetic-checks.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return
    if args.log_dir is None:
        parser.error("Укажите папку записи либо --self-test")
    pairs = load_capture_pairs(args.log_dir, args.allow_legacy_order)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    reports = []
    for pair in pairs:
        npz_path, metadata, pose = pair["file"], pair["metadata"], pair["pose"]
        analysis = analyze_voxels(npz_path, metadata, pose)
        analysis["source_file"] = str(npz_path.resolve())
        analysis["capture_pairing"] = pair["pairing"]
        analysis["capture_sync"] = {"method": "nearest_recorded_receive_time",
                                    "pose_minus_lidar_receive_s": pose["elapsed_s"]-metadata["elapsed_s"],
                                    "sensor_clock_synchronization_verified": False}
        lidar_stamp = metadata["message"]["data"].get("stamp")
        pose_stamp = pose["message"]["data"].get("header", {}).get("stamp")
        if isinstance(lidar_stamp, (int, float)) and isinstance(pose_stamp, dict):
            pose_seconds = pose_stamp.get("sec", 0) + pose_stamp.get("nanosec", 0) * 1e-9
            analysis["capture_sync"]["reported_lidar_minus_pose_stamp_s"] = lidar_stamp - pose_seconds
            analysis["capture_sync"]["warning"] = "Часы/точность timestamp датчиков не проверены; близость времени приёма не доказывает синхронность измерений"
        output_name = f"voxel-{args.log_dir.name}-{int(npz_path.stem[6:]):02d}"
        (args.output_dir/f"{output_name}.json").write_text(json.dumps(analysis, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        save_overlay(npz_path, metadata, pose, analysis, args.output_dir/f"{output_name}.png")
        points, _, _ = load_voxel_points(npz_path, metadata)
        np.savez_compressed(args.output_dir/f"{output_name}-points.npz", points=points)
        reports.append({"file": npz_path.name, "candidates": len(analysis["inclined_planes"]),
                        "floor": analysis["floor"], "map": analysis["map"]})
    print(json.dumps(reports, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
