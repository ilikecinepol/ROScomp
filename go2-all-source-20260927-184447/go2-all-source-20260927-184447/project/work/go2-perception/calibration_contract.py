"""Строгая геометрическая привязка для явно предоставленной калибровки.

Никаких параметров реального Go2 или предположений о высоте пола здесь нет.
Контракт JSON, версия 1:
  schema_version, width, height, K (3×3), D, distortion_model,
  T_camera_from_body (4×4, перенос в метрах), body_frame, pose_child_frame,
  camera_axes='x_right_y_down_z_forward',
  sources={intrinsics:{reference,sha256}, extrinsics:{reference,sha256}},
  validation={approved_for_robot:true, robot_id, camera_stream_id, method}.

Для pinhole D обязан быть пустым; для plumb_bob содержит ровно пять чисел
[k1,k2,p1,p2,k3]. T задаёт p_camera = R @ p_body + t в оптических осях OpenCV.
Проекция OpenCV не использует ненулевой skew, поэтому такой K отклоняется.

Флаги одобрения и ссылки на источники — утверждения доверенного вызывающего
кода. Этот модуль проверяет структуру и согласованность, но не доказывает
физическую точность калибровки и не читает файлы источников. Синтетические
проверки не являются одобрением для какого-либо реального робота.
"""

from __future__ import annotations

import math
import re

import cv2
import numpy as np


CONTRACT_VERSION = 1
_SHA256 = re.compile(r"^[0-9a-fA-F]{64}$")


def _blocked(reasons):
    return {"status": "blocked", "reasons": list(reasons)}


def _text(value):
    return isinstance(value, str) and bool(value.strip())


def _numeric(value, shape=None):
    """Булевы значения и числовые строки не являются числами калибровки."""
    try:
        if any(isinstance(v, (bool, np.bool_)) for v in np.asarray(value, dtype=object).flat):
            return None
        raw = np.asarray(value)
        if raw.dtype.kind not in "iuf" or (shape is not None and raw.shape != shape):
            return None
        result = np.asarray(value, dtype=np.float64)
        return result if np.all(np.isfinite(result)) else None
    except (ValueError, TypeError, OverflowError):
        return None


def _source_valid(value):
    return (isinstance(value, dict) and _text(value.get("reference"))
        and isinstance(value.get("sha256"), str) and bool(_SHA256.fullmatch(value["sha256"])))


def _frame_name(value):
    return (_text(value) and bool(re.fullmatch(r"[A-Za-z][A-Za-z0-9_/.-]*", value))
        and value.lower() not in ("unknown", "unverified", "none", "null", "tbd"))


def validate_calibration(calibration, *, robot_id=None, camera_stream_id=None,
                         image_size=None, pose_child_frame=None):
    """Проверяет JSON и соответствие контексту; отсутствие контекста даёт отказ.

    image_size — (width, height) фактического кадра. pose_child_frame должен
    прийти из подтверждённого контракта позы, а не угадываться по имени топика.
    """
    if not isinstance(calibration, dict):
        return _blocked(["Калибровка не предоставлена как объект JSON."])
    errors = []
    if type(calibration.get("schema_version")) is not int or calibration.get("schema_version") != CONTRACT_VERSION:
        errors.append("Неподдерживаемая или отсутствующая версия контракта калибровки.")
    width, height = calibration.get("width"), calibration.get("height")
    if any(not isinstance(v, int) or isinstance(v, bool) or v <= 0 for v in (width, height)):
        errors.append("width и height должны быть положительными целыми числами.")
    actual_size = _numeric(image_size, (2,))
    if actual_size is None or np.any(actual_size <= 0) or np.any(actual_size != np.floor(actual_size)):
        errors.append("Не задан точный размер фактического изображения.")
    elif (width, height) != tuple(actual_size):
        errors.append("Размер кадра не совпадает с размером калибровки; автоматическое масштабирование запрещено.")
    K = _numeric(calibration.get("K"), (3, 3))
    if K is None:
        errors.append("K должна быть конечной числовой матрицей 3×3.")
    elif K[0, 0] <= 0 or K[1, 1] <= 0 or not np.allclose(K[2], [0, 0, 1], atol=1e-9, rtol=0) or abs(K[1, 0]) > 1e-9 or abs(K[0, 1]) > 1e-9:
        errors.append("K должна иметь положительные fx/fy, нулевой skew и последнюю строку [0,0,1].")
    model = calibration.get("distortion_model")
    D = _numeric(calibration.get("D"))
    if model not in ("pinhole", "plumb_bob"):
        errors.append("Поддерживаются только явно заданные модели pinhole и plumb_bob.")
    if D is None or D.ndim != 1:
        errors.append("D должна быть одномерным конечным числовым массивом.")
    elif model == "pinhole" and D.size != 0:
        errors.append("Для pinhole требуется явный пустой D; ненулевая дисторсия не игнорируется.")
    elif model == "plumb_bob" and D.size != 5:
        errors.append("Для plumb_bob требуется D=[k1,k2,p1,p2,k3].")
    transform = _numeric(calibration.get("T_camera_from_body"), (4, 4))
    if transform is None:
        errors.append("T_camera_from_body должна быть конечной числовой матрицей 4×4.")
    else:
        rotation = transform[:3, :3]
        if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-9, rtol=0):
            errors.append("Последняя строка T_camera_from_body должна равняться [0,0,0,1].")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0) or not math.isclose(float(np.linalg.det(rotation)), 1., abs_tol=1e-6):
            errors.append("Вращение T_camera_from_body должно принадлежать SO(3): RᵀR=I и det(R)=+1.")
    if calibration.get("camera_axes") != "x_right_y_down_z_forward":
        errors.append("Не подтверждены оптические оси камеры x вправо, y вниз, z вперёд.")
    body = calibration.get("body_frame")
    declared_child = calibration.get("pose_child_frame")
    if not _frame_name(body) or not _frame_name(declared_child) or not _frame_name(pose_child_frame):
        errors.append("Не заданы body_frame и подтверждённый точный pose_child_frame.")
    elif not body == declared_child == pose_child_frame:
        errors.append("body_frame, pose_child_frame калибровки и дочерняя система фактической позы различаются.")
    sources = calibration.get("sources")
    for name in ("intrinsics", "extrinsics"):
        if not isinstance(sources, dict) or not _source_valid(sources.get(name)):
            errors.append(f"Для sources.{name} обязательны reference и SHA256 исходного свидетельства.")
    validation = calibration.get("validation")
    if not isinstance(validation, dict):
        errors.append("Отсутствует блок validation.")
    else:
        if validation.get("approved_for_robot") is not True:
            errors.append("Калибровка не одобрена для физического применения: approved_for_robot не равно true.")
        if not _text(robot_id) or not _text(validation.get("robot_id")) or robot_id != validation.get("robot_id"):
            errors.append("Не подтверждено точное соответствие калибровки идентификатору робота.")
        if not _text(camera_stream_id) or not _text(validation.get("camera_stream_id")) or camera_stream_id != validation.get("camera_stream_id"):
            errors.append("Не подтверждено точное соответствие калибровки видеопотоку.")
        if not _text(validation.get("method")):
            errors.append("Не описан способ проверки калибровки validation.method.")
    if errors:
        return _blocked(errors)
    return {"status": "valid", "reasons": [], "body_frame": body,
        "camera_stream_id": camera_stream_id, "robot_id": robot_id,
        "physical_accuracy_verified_by_this_function": False}


def project_body_points(points_body, calibration, *, robot_id=None, camera_stream_id=None,
                        image_size=None, pose_child_frame=None):
    """Проецирует метры из body_frame в пиксели только после строгой проверки.

    Для точек за камерой возвращает null вместо фиктивных пикселей. Отрицательные
    глубины и выход за кадр сохраняются как признаки, не как видимые препятствия.
    """
    validation = validate_calibration(calibration, robot_id=robot_id, camera_stream_id=camera_stream_id,
        image_size=image_size, pose_child_frame=pose_child_frame)
    if validation["status"] != "valid":
        return validation
    points = _numeric(points_body)
    if points is None or points.ndim != 2 or points.shape[1] != 3:
        return _blocked(["Точки должны быть конечным числовым массивом N×3 в метрах body_frame."])
    T = np.asarray(calibration["T_camera_from_body"], dtype=float)
    camera_points = points @ T[:3, :3].T + T[:3, 3]
    if not np.all(np.isfinite(camera_points)):
        return _blocked(["Преобразование точек дало нечисловой результат."])
    front = camera_points[:, 2] > 1e-9
    pixels = [None] * len(points)
    inside = [False] * len(points)
    if np.any(front):
        K = np.asarray(calibration["K"], dtype=float)
        D = np.asarray(calibration["D"], dtype=float)
        projected, _ = cv2.projectPoints(camera_points[front], np.zeros(3), np.zeros(3), K, D if D.size else None)
        for i, uv in zip(np.flatnonzero(front), projected.reshape(-1, 2)):
            if not np.all(np.isfinite(uv)):
                return _blocked(["Проекция нечисловая; калибровка или геометрия выходят за допустимый численный диапазон."])
            pixels[int(i)] = uv.tolist()
            inside[int(i)] = bool(0 <= uv[0] < calibration["width"] and 0 <= uv[1] < calibration["height"])
    return {"status": "ok", "reasons": [], "pixels": pixels, "camera_depth_m": camera_points[:, 2].tolist(),
        "in_front_of_camera": front.tolist(), "inside_image": inside, "body_frame": calibration["body_frame"],
        "camera_stream_id": camera_stream_id}


def camera_detection_to_metric_target(camera_detection, calibration, *, robot_id=None,
        camera_stream_id=None, image_size=None, pose_child_frame=None, plane_body=None, observation_id=None):
    """Пересекает луч центра предполагаемого входа с явно измеренной плоскостью.

    Дополнительно обязательны:
      camera_detection.observation_id == observation_id;
      plane_body={normal:[nx,ny,nz], offset_m:d, frame_id, observation_id,
                  robot_id, camera_stream_id, validated:true,
                  source:{reference,sha256}}.
    Уравнение плоскости: n·p + d = 0; n единичная, p в метрах body_frame.
    Плоскость не выводится из изображения и не заменяется предположением о поле.
    Допускается одна полная детекция с candidates или один объект entrance.
    Результат — геометрический кандидат; разрешением движения он не является.
    """
    validation = validate_calibration(calibration, robot_id=robot_id, camera_stream_id=camera_stream_id,
        image_size=image_size, pose_child_frame=pose_child_frame)
    if validation["status"] != "valid":
        return validation
    if not isinstance(camera_detection, dict):
        return _blocked(["Нет детекции камеры."])
    if not _text(observation_id) or camera_detection.get("observation_id") != observation_id:
        return _blocked(["Детекция не привязана к точному observation_id текущего измерения."])
    detected_size = camera_detection.get("image_size")
    if not isinstance(detected_size, dict) or (detected_size.get("width"), detected_size.get("height")) != (calibration["width"], calibration["height"]):
        return _blocked(["Детекция должна явно содержать совпадающие размеры исходного кадра."])
    candidate = camera_detection
    if "candidates" in camera_detection:
        candidates = camera_detection["candidates"]
        if not isinstance(candidates, list) or len(candidates) != 1 or not isinstance(candidates[0], dict):
            return _blocked(["Нужен ровно один явно выбранный кандидат входа."])
        candidate = candidates[0]
    entrance = candidate.get("entrance")
    center = _numeric(entrance.get("center_px") if isinstance(entrance, dict) else None, (2,))
    if center is None or not (0 <= center[0] < calibration["width"] and 0 <= center[1] < calibration["height"]):
        return _blocked(["Центр входа должен быть конечной точкой внутри исходного кадра."])
    if not isinstance(plane_body, dict):
        return _blocked(["Нет проверенной плоскости для пересечения луча; расстояние по умолчанию не используется."])
    normal = _numeric(plane_body.get("normal"), (3,))
    offset = _numeric(plane_body.get("offset_m"), ())
    plane_errors = []
    if normal is None or not math.isclose(float(np.linalg.norm(normal)), 1., abs_tol=1e-6):
        plane_errors.append("Нормаль плоскости должна быть конечной и единичной.")
    if offset is None:
        plane_errors.append("Смещение плоскости offset_m должно быть явно заданным конечным числом.")
    if plane_body.get("validated") is not True or not _source_valid(plane_body.get("source")):
        plane_errors.append("Плоскость требует validated=true и ссылки на проверенное измерение с SHA256.")
    for key, expected in (("frame_id", calibration["body_frame"]), ("robot_id", robot_id),
            ("camera_stream_id", camera_stream_id), ("observation_id", observation_id)):
        if plane_body.get(key) != expected:
            plane_errors.append(f"Плоскость не соответствует текущему {key}.")
    if plane_errors:
        return _blocked(plane_errors)
    K = np.asarray(calibration["K"], dtype=float)
    D = np.asarray(calibration["D"], dtype=float)
    ray_xy = cv2.undistortPoints(center.reshape(1, 1, 2), K, D if D.size else None).reshape(2)
    if not np.all(np.isfinite(ray_xy)):
        return _blocked(["Нельзя получить конечный оптический луч."])
    T = np.asarray(calibration["T_camera_from_body"], dtype=float)
    camera_origin_body = -T[:3, :3].T @ T[:3, 3]
    ray_body = T[:3, :3].T @ np.array([ray_xy[0], ray_xy[1], 1.])
    ray_body /= np.linalg.norm(ray_body)
    denominator = float(normal @ ray_body)
    if abs(denominator) < 1e-6:
        return _blocked(["Луч почти параллелен плоскости; метрическая точка неустойчива."])
    distance = -(float(normal @ camera_origin_body) + float(offset)) / denominator
    if not math.isfinite(distance) or distance <= 1e-9:
        return _blocked(["Пересечение плоскости лежит за камерой или не имеет конечной положительной дальности."])
    point_body = camera_origin_body + distance * ray_body
    if not np.all(np.isfinite(point_body)):
        return _blocked(["Получена нечисловая метрическая точка."])
    reprojection = project_body_points([point_body.tolist()], calibration, robot_id=robot_id,
        camera_stream_id=camera_stream_id, image_size=image_size, pose_child_frame=pose_child_frame)
    if reprojection["status"] != "ok" or not reprojection["in_front_of_camera"][0]:
        return _blocked(["Контрольная проекция метрической точки не удалась."])
    residual = float(np.linalg.norm(np.asarray(reprojection["pixels"][0]) - center))
    if residual > .25:
        return _blocked(["Обратная коррекция дисторсии не прошла проверку повторной проекцией (ошибка >0,25 пикселя)."])
    return {"status": "ok", "reasons": [], "target_kind": "geometric_candidate_not_motion_authorization",
        "point_body_m": point_body.tolist(), "body_frame": calibration["body_frame"],
        "distance_from_camera_m": float(distance), "reprojection_error_px": residual,
        "observation_id": observation_id, "camera_stream_id": camera_stream_id,
        "requires_traversability_and_target_association_validation": True}
