"""Искусственные геометрические проверки; реальных параметров Go2 здесь нет."""

import copy
import hashlib
import json
import platform
from pathlib import Path

import cv2
import numpy as np

from calibration_contract import (camera_detection_to_metric_target,
    project_body_points, validate_calibration)


ROOT = Path(__file__).resolve().parents[2]
CONTEXT = {"robot_id": "synthetic-only-robot", "camera_stream_id": "synthetic-only-camera",
    "image_size": (640, 480), "pose_child_frame": "synthetic_body"}
SOURCE = {"reference": "ИСКУССТВЕННАЯ ГЕОМЕТРИЯ ТЕСТА, НЕ КАЛИБРОВКА РОБОТА",
    "sha256": hashlib.sha256(b"synthetic calibration contract geometry v1").hexdigest()}


def calibration():
    # Искусственная камера в точке body=(0.2,0,0.4), смотрящая вдоль +X.
    return {"schema_version": 1, "width": 640, "height": 480,
        "K": [[400., 0., 320.], [0., 400., 240.], [0., 0., 1.]], "D": [], "distortion_model": "pinhole",
        "T_camera_from_body": [[0., -1., 0., 0.], [0., 0., -1., .4], [1., 0., 0., -.2], [0., 0., 0., 1.]],
        "body_frame": "synthetic_body", "pose_child_frame": "synthetic_body",
        "camera_axes": "x_right_y_down_z_forward", "sources": {"intrinsics": dict(SOURCE), "extrinsics": dict(SOURCE)},
        "validation": {"approved_for_robot": True, "robot_id": CONTEXT["robot_id"],
            "camera_stream_id": CONTEXT["camera_stream_id"], "method": "Только аналитическая искусственная геометрия теста; физического одобрения нет."}}


def detection():
    return {"observation_id": "synthetic-observation-1", "image_size": {"width": 640, "height": 480},
        "candidates": [{"entrance": {"center_px": [360., 300.]}}]}


def plane():
    return {"normal": [0., 0., 1.], "offset_m": -.1, "frame_id": "synthetic_body",
        "observation_id": "synthetic-observation-1", "robot_id": CONTEXT["robot_id"],
        "camera_stream_id": CONTEXT["camera_stream_id"], "validated": True, "source": dict(SOURCE)}


def require(condition, message="Не выполнено ожидаемое условие"):
    if not condition:
        raise AssertionError(message)


def expect_blocked(result):
    require(result["status"] == "blocked", repr(result))
    require(bool(result.get("reasons")), "Отказ без причины")
    require(not any(key in result for key in ("point_body_m", "distance_from_camera_m", "pixels")), "При отказе выдана метрическая геометрия")


def metric(c=None, d=None, p=None, **overrides):
    args = {**CONTEXT, "plane_body": plane() if p is None else p, "observation_id": "synthetic-observation-1"}
    args.update(overrides)
    return camera_detection_to_metric_target(detection() if d is None else d, calibration() if c is None else c, **args)


def main():
    results = []
    def check(name, procedure):
        try:
            procedure()
            results.append({"case": name, "passed": True})
        except Exception as exc:
            results.append({"case": name, "passed": False, "error": f"{type(exc).__name__}: {exc}"})

    check("валидный искусственный контракт", lambda: require(validate_calibration(calibration(), **CONTEXT)["status"] == "valid"))
    check("без калибровки метрическая привязка запрещена", lambda: expect_blocked(camera_detection_to_metric_target(detection(), None, **CONTEXT)))
    check("без калибровки проекция запрещена", lambda: expect_blocked(project_body_points([[1, 0, 0]], None, **CONTEXT)))
    check("без фактического контекста проверка запрещена", lambda: expect_blocked(validate_calibration(calibration())))

    def analytic_projection():
        result = project_body_points([[2.2, -.2, .1], [2.2, 0., .4], [-1., 0., .4], [.2, 0., .4], [1.2, -10., .4]], calibration(), **CONTEXT)
        require(result["status"] == "ok")
        np.testing.assert_allclose(result["pixels"][:2], [[360., 300.], [320., 240.]], atol=1e-10)
        require(result["pixels"][2] is None and result["pixels"][3] is None)
        require(result["inside_image"] == [True, True, False, False, False])
        require(result["in_front_of_camera"] == [True, True, False, False, True])
    check("известная аналитическая проекция, точка за камерой, нулевая глубина и выход за кадр", analytic_projection)

    def analytical_distortion():
        c = calibration()
        c.update({"distortion_model": "plumb_bob", "D": [.1, .01, .002, -.003, .001]})
        x, y = .1, .15
        radius = x*x+y*y
        radial = 1+.1*radius+.01*radius**2+.001*radius**3
        xd = x*radial+2*.002*x*y-.003*(radius+2*x*x)
        yd = y*radial+.002*(radius+2*y*y)+2*-.003*x*y
        result = project_body_points([[2.2, -.2, .1]], c, **CONTEXT)
        require(result["status"] == "ok")
        np.testing.assert_allclose(result["pixels"][0], [400*xd+320, 400*yd+240], atol=1e-9)
        d = detection()
        d["candidates"][0]["entrance"]["center_px"] = result["pixels"][0]
        target = metric(c=c, d=d)
        require(target["status"] == "ok", repr(target))
        np.testing.assert_allclose(target["point_body_m"], [2.2, -.2, .1], atol=1e-8)
    check("plumb_bob: аналитическая дисторсия и обратная привязка", analytical_distortion)

    def analytic_intersection():
        result = metric()
        require(result["status"] == "ok", repr(result))
        np.testing.assert_allclose(result["point_body_m"], [2.2, -.2, .1], atol=1e-10)
        np.testing.assert_allclose(result["distance_from_camera_m"], np.linalg.norm([2., -.2, -.3]), atol=1e-10)
        require(result["requires_traversability_and_target_association_validation"] is True)
        json.dumps(result, allow_nan=False)
    check("луч пересекает явно заданную плоскость в известной точке", analytic_intersection)

    for name, path, value in [
        ("нет положительного fx", ["K", 0, 0], 0),
        ("матрица K содержит NaN", ["K", 0, 0], float("nan")),
        ("неподдерживаемый skew", ["K", 0, 1], 1.),
        ("неподдерживаемая модель fisheye", ["distortion_model"], "fisheye"),
        ("нет D", ["D"], None),
        ("pinhole не игнорирует переданные коэффициенты", ["D"], [.1, 0., 0., 0., 0.]),
        ("нечисловая строка внутри K", ["K", 0, 0], "400"),
        ("булево внутри K", ["K", 0, 0], True),
        ("неоднородная последняя строка T", ["T_camera_from_body", 3, 3], 2.),
        ("неортонормальное вращение", ["T_camera_from_body", 0, 1], -2.),
        ("перенос содержит Infinity", ["T_camera_from_body", 0, 3], float("inf")),
        ("неподтверждённые оптические оси", ["camera_axes"], "body_axes"),
        ("несовпадение дочерней системы", ["pose_child_frame"], "lidar"),
        ("флаг одобрения false", ["validation", "approved_for_robot"], False),
        ("число вместо флага одобрения", ["validation", "approved_for_robot"], 1),
        ("другой робот", ["validation", "robot_id"], "another-robot"),
        ("другой видеопоток", ["validation", "camera_stream_id"], "another-stream"),
        ("нет способа физической проверки", ["validation", "method"], ""),
        ("нет источника intrinsics", ["sources", "intrinsics"], {}),
        ("нет SHA256 extrinsics", ["sources", "extrinsics", "sha256"], ""),
        ("другой размер изображения", ["width"], 1280),
        ("булево вместо ширины", ["width"], True),
        ("неизвестная версия", ["schema_version"], 2),
    ]:
        def mutation(path=path, value=value):
            c = calibration()
            node = c
            for part in path[:-1]:
                node = node[part]
            node[path[-1]] = value
            expect_blocked(validate_calibration(c, **CONTEXT))
            expect_blocked(project_body_points([[2.2, -.2, .1]], c, **CONTEXT))
            expect_blocked(metric(c=c))
        check(name, mutation)

    def reflection():
        c = calibration()
        c["T_camera_from_body"][0][1] = 1.
        expect_blocked(validate_calibration(c, **CONTEXT))
    check("ортогональное отражение det=-1 отвергается", reflection)

    def unknown_frame():
        c = calibration()
        c["body_frame"] = c["pose_child_frame"] = "unknown"
        expect_blocked(validate_calibration(c, **{**CONTEXT,"pose_child_frame":"unknown"}))
    check("слово unknown не является доказательством системы координат", unknown_frame)

    def d_count():
        c = calibration()
        c.update({"distortion_model": "plumb_bob", "D": [0., 0., 0., 0.]})
        expect_blocked(validate_calibration(c, **CONTEXT))
    check("plumb_bob требует пять коэффициентов", d_count)
    check("нет плоскости — нет расстояния по умолчанию", lambda: expect_blocked(camera_detection_to_metric_target(detection(), calibration(), **CONTEXT, observation_id="synthetic-observation-1")))
    check("нет привязки наблюдения", lambda: expect_blocked(metric(observation_id=None)))
    check("точки NaN запрещены", lambda: expect_blocked(project_body_points([[1., float('nan'), 0.]], calibration(), **CONTEXT)))
    check("плоский массив вместо Nx3 запрещён", lambda: expect_blocked(project_body_points([1., 0., 0.], calibration(), **CONTEXT)))

    for name, key, value in [
        ("неединичная нормаль", "normal", [0., 0., 2.]),
        ("смещение плоскости NaN", "offset_m", float("nan")),
        ("непроверенная плоскость", "validated", False),
        ("неизвестный источник плоскости", "source", {}),
        ("плоскость другой системы", "frame_id", "odom"),
        ("плоскость другого робота", "robot_id", "another-robot"),
        ("плоскость другого потока", "camera_stream_id", "another-stream"),
        ("плоскость другого измерения", "observation_id", "older-observation"),
        ("плоскость за направлением луча", "offset_m", -1.),
    ]:
        def plane_mutation(key=key, value=value):
            p = plane()
            p[key] = value
            expect_blocked(metric(p=p))
        check(name, plane_mutation)

    def parallel():
        d = detection()
        d["candidates"][0]["entrance"]["center_px"] = [320.,240.]
        expect_blocked(metric(d=d))
    check("параллельный плоскости луч не имеет придуманной дальности", parallel)
    def ambiguous():
        d = detection()
        d["candidates"].append(copy.deepcopy(d["candidates"][0]))
        expect_blocked(metric(d=d))
    check("несколько кандидатов требуют явного выбора", ambiguous)
    def outside():
        d = detection()
        d["candidates"][0]["entrance"]["center_px"] = [-1.,300.]
        expect_blocked(metric(d=d))
    check("пиксель вне кадра запрещён", outside)
    def wrong_detection_size():
        d = detection()
        d["image_size"]["width"] = 1280
        expect_blocked(metric(d=d))
    check("размер детекции не подменяет размер калибровки", wrong_detection_size)

    report = {"status": "passed" if all(c["passed"] for c in results) else "failed",
        "passed": sum(c["passed"] for c in results), "total": len(results),
        "scope": "Только искусственная геометрия и отказные проверки. Не доказывает существование, точность или одобрение калибровки реального Go2.",
        "approves_any_real_robot": False, "network_or_motion_used": False,
        "python_version": platform.python_version(), "opencv_version": cv2.__version__,
        "module_sha256": hashlib.sha256((Path(__file__).parent/'calibration_contract.py').read_bytes()).hexdigest(), "cases": results}
    out = ROOT/'outputs/go2-perception/coordinate-audit/calibration-contract-tests.json'
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({"status":report["status"],"passed":report["passed"],"total":report["total"]}))
    for case in results:
        if not case["passed"]:
            print(case)
    if report["status"] != "passed":
        raise SystemExit(1)


if __name__ == '__main__':
    main()
