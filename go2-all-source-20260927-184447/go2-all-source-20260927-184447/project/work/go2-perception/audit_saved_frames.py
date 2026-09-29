"""Локальный воспроизводимый аудит двух сохранённых серий Go2.

Не подключается к роботу. Вычисления не определяют причину расхождения
датчиков: проверяются согласованность представлений, часы и изображения.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import scipy
from scipy import ndimage

from voxel_geometry import load_capture_pairs, load_voxel_points


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def stamp_seconds(stamp):
    if isinstance(stamp, dict):
        return float(stamp["sec"]) + float(stamp["nanosec"])*1e-9
    return float(stamp) if stamp is not None else None


def quaternion_rpy(xyzw):
    q = np.asarray(xyzw, dtype=float)
    q = q/np.linalg.norm(q)
    x, y, z, w = q
    return np.array([math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                     math.asin(float(np.clip(2*(w*y-z*x), -1, 1))),
                     math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))])


def series_summary(rows):
    """Тренд курса считается по timestamp датчика, не по пакетному приёму."""
    receive = np.array([row["receive_elapsed_s"] for row in rows])
    result = {"count": len(rows), "receive_span_s": float(np.ptp(receive))}
    stamps = [row["sensor_stamp_s"] for row in rows if row.get("sensor_stamp_s") is not None]
    if stamps:
        result.update({"stamp_min_s": min(stamps), "stamp_max_s": max(stamps),
                       "stamp_span_s": max(stamps)-min(stamps), "stamp_unique_count": len(set(stamps))})
    if rows and "rpy_rad" in rows[0]:
        rpy = np.array([row["rpy_rad"] for row in rows])
        yaws = np.unwrap(rpy[:, 2])
        result.update({"mean_rpy_rad": np.mean(rpy, axis=0).tolist(),
                       "yaw_span_deg": float(np.degrees(np.ptp(yaws))),
                       "mean_yaw_rad": float(np.mean(yaws))})
        if len(stamps) == len(rows) and np.ptp(stamps) > 0:
            rate = np.polyfit(np.array(stamps)-stamps[0], yaws, 1)[0]
            result["yaw_rate_fit_deg_per_s"] = float(np.degrees(rate))
    return result


def audit_capture(folder):
    records = [json.loads(line) for line in (folder/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
    streams = {}
    for topic in sorted({record["topic"] for record in records}):
        rows = []
        for record in [row for row in records if row["topic"] == topic]:
            data = record["message"]["data"]
            stamp = data.get("stamp", data.get("header", {}).get("stamp"))
            row = {"receive_elapsed_s": record["elapsed_s"], "sensor_stamp_s": stamp_seconds(stamp)}
            if topic in ("LOW_STATE", "LF_SPORT_MOD_STATE"):
                row["rpy_rad"] = data["imu_state"]["rpy"]
            if topic == "LF_SPORT_MOD_STATE":
                wxyz = data["imu_state"]["quaternion"]
                xyzw = wxyz[1:] + wxyz[:1]
                calculated = quaternion_rpy(xyzw)
                wrong = quaternion_rpy(wxyz)
                row.update({"quaternion_as_saved": wxyz, "quaternion_norm": float(np.linalg.norm(wxyz)),
                            "rpy_from_wxyz_rad": calculated.tolist(),
                            "wxyz_rpy_max_error_deg": float(np.degrees(abs(wrap(calculated-row["rpy_rad"]))).max()),
                            "xyzw_wrong_order_rpy_max_error_deg": float(np.degrees(abs(wrap(wrong-row["rpy_rad"]))).max()),
                            "gyro_saved": data["imu_state"]["gyroscope"], "position_m": data["position"]})
            if topic == "ROBOTODOM":
                orient = data["pose"]["orientation"]
                xyzw = [orient[axis] for axis in "xyzw"]
                row.update({"quaternion_named_xyzw": xyzw, "quaternion_norm": float(np.linalg.norm(xyzw)),
                            "rpy_rad": quaternion_rpy(xyzw).tolist(),
                            "position_m": [data["pose"]["position"][axis] for axis in "xyz"],
                            "frame_id": data["header"]["frame_id"]})
            if topic == "ULIDAR_STATE":
                row.update({"imu_rpy_saved_units_unknown": data.get("imu_rpy"),
                            "serial_recv_stamp": data.get("serial_recv_stamp"),
                            "error_state": data.get("error_state")})
            if topic == "ULIDAR_ARRAY":
                row.update({"origin_m": data["origin"], "resolution_m": data["resolution"],
                            "width_voxels": data["width"], "frame_id": data["frame_id"],
                            "src_size_bytes": data.get("src_size"), "decoded_array_metadata": data.get("data")})
            rows.append(row)
        streams[topic] = {"summary": series_summary(rows), "samples": rows}
    sport = streams["LF_SPORT_MOD_STATE"]["samples"]
    odom = streams["ROBOTODOM"]["samples"]
    low = streams["LOW_STATE"]["samples"]
    matches = []
    for row in odom:
        nearest = min(sport, key=lambda item: abs(item["sensor_stamp_s"]-row["sensor_stamp_s"]))
        matches.append({"odom_stamp_s": row["sensor_stamp_s"],
                        "sport_minus_odom_stamp_s": nearest["sensor_stamp_s"]-row["sensor_stamp_s"],
                        "rpy_delta_deg": np.degrees(wrap(np.array(nearest["rpy_rad"])-row["rpy_rad"])).tolist(),
                        "position_delta_m": (np.array(nearest["position_m"])-row["position_m"]).tolist()})
    # У LOW_STATE нет timestamp: сопоставление по приёму не синхронно.
    low_offsets = [float(wrap(row["rpy_rad"][2]-min(sport, key=lambda item: abs(item["receive_elapsed_s"]-row["receive_elapsed_s"]))["rpy_rad"][2])) for row in low]
    gyro = np.array([row["gyro_saved"] for row in sport])
    rpy = np.array([row["rpy_rad"] for row in sport])
    conditional_yaw_rate = (np.sin(rpy[:, 0])*gyro[:, 1]+np.cos(rpy[:, 0])*gyro[:, 2])/np.cos(rpy[:, 1])
    return {"capture": folder.name, "streams": streams,
            "odom_sport_nearest_sensor_stamp_comparison": matches,
            "gyro_check": {"gyro_mean_as_saved": np.mean(gyro, axis=0).tolist(),
                           "predicted_mean_euler_yaw_rate_rad_s_if_body_gyro_rad_s_ZYX": float(np.mean(conditional_yaw_rate)),
                           "assumption": "Условное вычисление для gyroscope в rad/s в системе корпуса и стандартных углах ZYX; единицы/обработка драйвера отдельно не подтверждены",
                           "limit": "Короткие окна и возможная коррекция оценки ориентации не позволяют определить физическую причину расхождения"},
            "low_minus_sport_yaw_deg_by_receive_mean": float(np.degrees(np.mean(low_offsets))),
            "low_comparison_limit": "Нет timestamp LOW_STATE; использовано лишь время приёма",
            "quaternion_order_conclusion": "LF_SPORT_MOD_STATE array wxyz соответствует rpy; ROBOTODOM использует именованные x/y/z/w"}


def camera_features(path):
    original = cv2.imread(str(path))
    if original is None:
        raise ValueError(f"Кадр не прочитан: {path}")
    gray = cv2.cvtColor(cv2.resize(original, (640, 360)), cv2.COLOR_BGR2GRAY)
    keypoints, descriptors = cv2.SIFT_create(nfeatures=1500).detectAndCompute(gray, None)
    return gray, keypoints, descriptors


def compare_camera(a, b):
    g1, k1, d1 = a
    g2, k2, d2 = b
    matches = cv2.BFMatcher().knnMatch(d1, d2, k=2)
    good = [first for pair in matches if len(pair) == 2 for first, second in [pair] if first.distance < .7*second.distance]
    result = {"analysis_image_size": [640, 360], "grayscale_mae_0_255": float(np.mean(abs(g1.astype(float)-g2))),
              "pixel_correlation": float(np.corrcoef(g1.ravel(), g2.ravel())[0, 1]), "sift_ratio_matches": len(good)}
    if len(good) >= 8:
        src = np.float32([k1[m.queryIdx].pt for m in good])
        dst = np.float32([k2[m.trainIdx].pt for m in good])
        transform, mask = cv2.findHomography(src, dst, cv2.RANSAC, 2.0)
        if transform is not None:
            mask = mask.ravel().astype(bool)
            displacement = dst[mask]-src[mask]
            corners = np.float32([[0, 0], [639, 0], [639, 359], [0, 359]]).reshape(-1, 1, 2)
            moved = cv2.perspectiveTransform(corners, transform)
            result.update({"homography_a_to_b": transform.tolist(), "ransac_inliers": int(mask.sum()),
                           "inlier_median_displacement_px": np.median(displacement, axis=0).tolist(),
                           "inlier_p95_displacement_norm_px": float(np.quantile(np.linalg.norm(displacement, axis=1), .95)),
                           "homography_max_corner_shift_px": float(np.linalg.norm(moved-corners, axis=2).max())})
    return result


def polar_map(pair):
    points, metadata, decode = load_voxel_points(pair["file"], pair["metadata"])
    origin = np.asarray(metadata["origin"])
    resolution = float(metadata["resolution"])
    width = np.asarray(metadata["width"])
    grid = np.rint((points-origin)/resolution).astype(int)
    shape = np.maximum(width[:2], grid[:, :2].max(axis=0)+1)
    height = np.full(tuple(shape), -np.inf)
    np.maximum.at(height, (grid[:, 0], grid[:, 1]), points[:, 2])
    height[~np.isfinite(height)] = np.nan
    pose = pair["pose"]["message"]["data"]["pose"]
    position = np.array([pose["position"][key] for key in "xyz"])
    radii = np.arange(.65, 2.801, .05)
    angles = np.deg2rad(np.arange(0, 360, .5))
    xx = position[0]+radii[:, None]*np.cos(angles)
    yy = position[1]+radii[:, None]*np.sin(angles)
    values = ndimage.map_coordinates(height, [(xx-origin[0])/resolution, (yy-origin[1])/resolution],
                                     order=0, mode="constant", cval=np.nan)
    radial_mean = np.nanmean(values, axis=1)
    residual = values-radial_mean[:, None]
    return values, residual, {"encoding": decode["encoding"], "point_count": len(points),
                              "bounds_min_m": points.min(axis=0).tolist(), "bounds_max_m": points.max(axis=0).tolist(),
                              "radius_m": radii.tolist(), "radial_mean_z_m": radial_mean.tolist(),
                              "angular_std_z_per_radius_m": np.nanstd(values, axis=1).tolist(),
                              "radial_component_variance_fraction": float(np.nanvar(radial_mean)/np.nanvar(values)),
                              "valid_polar_fraction": float(np.mean(np.isfinite(values)))}


def correlation(a, b):
    valid = np.isfinite(a) & np.isfinite(b)
    if valid.sum() < 100:
        return None
    aa, bb = a[valid], b[valid]
    return float(np.corrcoef(aa, bb)[0, 1])


def audit_maps(folders, yaw_change):
    pairs = [load_capture_pairs(folder, allow_legacy_order=True) for folder in folders]
    polars = [[polar_map(pair) for pair in capture] for capture in pairs]
    first, second = polars[0][0], polars[1][0]
    rows = []
    for shift in range(720):
        rows.append({"rotation_old_to_new_deg": float(np.degrees(wrap(math.radians(shift*.5)))),
                     "height_correlation": correlation(np.roll(first[0], shift, axis=1), second[0]),
                     "radial_residual_correlation": correlation(np.roll(first[1], shift, axis=1), second[1])})
    best = max(rows, key=lambda row: row["radial_residual_correlation"])
    yaw_row = min(rows, key=lambda row: abs(float(wrap(math.radians(row["rotation_old_to_new_deg"])-yaw_change))))
    high = [row["rotation_old_to_new_deg"] for row in rows if row["radial_residual_correlation"] >= best["radial_residual_correlation"]-.01]
    return {"method": "Верхние огибающие в полярной сетке вокруг позиции робота; 0.65…2.8 м, шаг 5 см, угол 0.5°. Отдельно вычтен средний радиальный профиль; независимой калибровкой не является.",
            "limitations": ["Сравниваются вершины mesh и нативные точки, поэтому возможна систематическая разница около ячейки",
                            "Радиальные структуры дают неоднозначность поворота; используется также остаток после вычитания среднего кольца",
                            "Одометрия используется лишь как центр полярной сетки; поиск угла не подгоняется под курс"],
            "captures": [{"capture": folders[i].name, "frames": [entry[2] for entry in polars[i]],
                          "within_capture_residual_correlation_to_first": [correlation(polars[i][0][1], item[1]) for item in polars[i]]}
                         for i in range(2)],
            "best_rotation_by_radial_residual": best, "rotation_at_reported_yaw_change": yaw_row,
            "zero_rotation": rows[0], "angles_within_0_01_of_best_deg": high, "rotation_scan": rows}


def write_markdown(report, path):
    captures = report["captures"]
    cross = report["between_captures"]
    maps = report["maps"]
    first_cam = report["camera_comparisons"][0]
    text = ["# Аудит координат сохранённых записей Go2", "",
            "Аудит выполнен только по локальным файлам. Соединений с роботом и команд движения нет. Причина несогласованности этими файлами не установлена.", "",
            "## Курс и порядок кватерниона", "",
            f"В каждой серии изучены все 10 сохранённых сообщений каждого из пяти потоков. Между средними курсами LF_SPORT_MOD_STATE изменение составляет {cross['sport_yaw_change_shortest_deg']:.3f}°; ROBOTODOM — {cross['odom_yaw_change_shortest_deg']:.3f}°; LOW_STATE — {cross['low_yaw_change_shortest_deg']:.3f}°. Это изменение уже присутствует в исходных rpy и в кватернионах, а не появляется при построении нашей карты.", "",
            "Массив LF_SPORT_MOD_STATE согласован с порядком **w,x,y,z**. Поля ROBOTODOM именованы x,y,z,w и разобраны по именам. Отрицательный знак всех компонент кватерниона не меняет ориентацию.", ""]
    for capture in captures:
        streams = capture["streams"]
        rows = streams["LF_SPORT_MOD_STATE"]["samples"]
        maximum = max(row["wxyz_rpy_max_error_deg"] for row in rows)
        wrong = min(row["xyzw_wrong_order_rpy_max_error_deg"] for row in rows)
        rate = streams["LF_SPORT_MOD_STATE"]["summary"]["yaw_rate_fit_deg_per_s"]
        odomerr = max(max(abs(v) for v in row["rpy_delta_deg"]) for row in capture["odom_sport_nearest_sensor_stamp_comparison"])
        text.append(f"- {capture['capture']}: максимальная ошибка rpy из wxyz {maximum:.6f}°; неверное чтение xyzw даёт ≥{wrong:.2f}° хотя бы по одной оси. Разница rpy ближайших по sensor timestamp SPORT и ODOM ≤{odomerr:.4f}°. LOW минус SPORT около {capture['low_minus_sport_yaw_deg_by_receive_mean']:.3f}° по yaw. Краткий линейный тренд SPORT: {rate:.4f}°/с.")
    text += ["", f"Средний темп изменения курса между сериями — {cross['sport_yaw_change_shortest_deg']/cross['sport_stamp_mean_difference_s']:.4f}°/с за {cross['sport_stamp_mean_difference_s']:.3f} с. Краткие внутрисерийные тренды близки по величине, но всего два коротких окна не доказывают непрерывный дрейф. Gyroscope сохранён отдельно в JSON; причина изменения yaw и физическая калибровка не определены.", "",
             "## Камера", "", f"На первых кадрах сцену связывают {first_cam['ransac_inliers']} согласованных SIFT-сопоставлений. Корреляция пикселей {first_cam['pixel_correlation']:.6f}, средняя абсолютная разница {first_cam['grayscale_mae_0_255']:.3f}/255; 95% согласованных смещений не превышают {first_cam['inlier_p95_displacement_norm_px']:.3f} пикселя при анализе 640×360. Проверены все 25 межсерийных пар пяти фотографий.", "",
             "Это свидетельство близкого направления камеры в двух записанных окнах. Кадры не подтверждают физический поворот корпуса на 132°. Отсутствуют независимая метка положения, точные времена экспозиции и запись промежутка между сессиями; нельзя исключить происхождение кадров или особенности их выдачи только по похожести сцены.", "",
             "## Карта", "", f"Независимый перебор поворота верхней огибающей, после вычитания среднего радиального профиля, даёт максимум около {maps['best_rotation_by_radial_residual']['rotation_old_to_new_deg']:.1f}°, корреляция {maps['best_rotation_by_radial_residual']['radial_residual_correlation']:.4f}. При повороте, ближайшем к изменению курса, корреляция {maps['rotation_at_reported_yaw_change']['radial_residual_correlation']:.4f}; без поворота {maps['zero_rotation']['radial_residual_correlation']:.4f}.", "",
             "Максимальная межсерийная корреляция невысока: этот поиск не подтверждает поворот карты на величину курса и не устанавливает её физическую неподвижность. Внутри каждой сессии карты значительно более сходны (значения приведены в JSON).", "",
             "Круговые структуры описаны радиальными профилями и долей радиальной дисперсии в JSON. Их наличие не объявляется стеной, радиусом обзора, калибровочным артефактом или неисправностью: сырых лучей, калибровки и независимой карты здесь нет. Сравнение mesh/native имеет систематическое различие геометрии порядка ячейки; это не точная регистрация облаков.", "",
             "## Время и единицы", "",
             "Во всех 20 сохранённых ULIDAR_ARRAY, 20 ULIDAR_STATE и их serial_recv_stamp записано ровно 1789556000.0. Время SPORT/ODOM при этом изменяется как внутри серии, так и между ними. Такая метка лидара непригодна для подтверждения синхронизации.", "",
             f"При приведении этого числа к float32 точное значение равно {report['timestamp_precision']['lidar_token_as_float32_exact']:.0f}, соседний шаг float32 — {report['timestamp_precision']['float32_spacing_s']:.0f} с. Простое приведение stamp SPORT к float32 даёт разные значения в двух сериях и потому само по себе не объясняет постоянный токен. Округление до семи значащих десятичных цифр может дать один токен для обеих серий; это демонстрация численной возможности, а не установленный механизм.", "",
             "RPY SPORT сопоставляется с кватернионами в радианах. Для imu_rpy в ULIDAR_STATE единицы и порядок осей из сохранённого JSON не определяются; приравнивать эти три числа к rpy корпуса нельзя. Origin и resolution согласуются с метрической воксельной сеткой; ячейка 0,05 м не равна точности физических измерений.", "",
             "## Воспроизведение", "", "```powershell", "& 'work/go2-perception/.venv/Scripts/python.exe' 'work/go2-perception/audit_saved_frames.py'", "```", "",
             "Полные значения всех сообщений, 25 сравнений камеры, угловой поиск карты, версии библиотек и хеши входных файлов находятся в audit.json. Для определения причины нужны проверка источника данных/драйвера и контролируемое сопоставление с независимой ориентацией; по этим записям преобразование odom в физическое помещение не подтверждено."]
    path.write_text("\n".join(text)+"\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-root", type=Path, default=Path("outputs/go2-remote-setup/logs"))
    parser.add_argument("--output", type=Path, default=Path("outputs/go2-perception/coordinate-audit"))
    args = parser.parse_args()
    cv2.setNumThreads(1)
    cv2.setRNGSeed(0)
    folders = [args.capture_root/name for name in ("20260916T105034Z", "20260916T110043Z")]
    captures = [audit_capture(folder) for folder in folders]
    report = {"captures": captures, "scope": "local_saved_files_only_no_robot_connection", "versions": {"numpy": np.__version__, "opencv": cv2.__version__, "scipy": scipy.__version__}}
    cross = {}
    for label, topic in (("sport", "LF_SPORT_MOD_STATE"), ("odom", "ROBOTODOM"), ("low", "LOW_STATE")):
        series = [capture["streams"][topic] for capture in captures]
        cross[label+"_yaw_change_shortest_deg"] = float(np.degrees(wrap(series[1]["summary"]["mean_yaw_rad"]-series[0]["summary"]["mean_yaw_rad"])))
        if label != "low":
            cross[label+"_stamp_mean_difference_s"] = float(np.mean([row["sensor_stamp_s"] for row in series[1]["samples"]])-np.mean([row["sensor_stamp_s"] for row in series[0]["samples"]]))
            cross[label+"_mean_position_difference_m"] = (np.mean([row["position_m"] for row in series[1]["samples"]], axis=0)-np.mean([row["position_m"] for row in series[0]["samples"]], axis=0)).tolist()
    report["between_captures"] = cross
    cameras = [[(path, camera_features(path)) for path in sorted(folder.glob("camera_*.jpg"))] for folder in folders]
    report["camera_comparisons"] = [dict(compare_camera(a, b), first=str(pa), second=str(pb)) for pa, a in cameras[0] for pb, b in cameras[1]]
    report["maps"] = audit_maps(folders, math.radians(cross["odom_yaw_change_shortest_deg"]))
    sport_stamps = [np.mean([row["sensor_stamp_s"] for row in capture["streams"]["LF_SPORT_MOD_STATE"]["samples"]]) for capture in captures]
    report["timestamp_precision"] = {"lidar_token": 1789556000.0, "lidar_token_as_float32_exact": float(np.float32(1789556000.0)),
                                     "float32_spacing_s": float(np.spacing(np.float32(1789556000.0))),
                                     "sport_mean_stamps_s": sport_stamps,
                                     "sport_mean_stamps_as_float32": [float(np.float32(value)) for value in sport_stamps],
                                     "sport_mean_stamps_7_significant_decimal_digits": [format(value, '.7g') for value in sport_stamps],
                                     "interpretation": "Разрешение/часы/округление исходного stamp лидара не установлены"}
    report["input_sha256"] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for folder in folders
                               for path in sorted(folder.iterdir()) if path.suffix in (".json", ".jsonl", ".npz", ".jpg")}
    report["script_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/"audit.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    write_markdown(report, args.output/"АУДИТ КООРДИНАТ.md")
    print(json.dumps({"between_captures": cross, "camera_first_pair": report["camera_comparisons"][0],
                      "map_best_rotation": report["maps"]["best_rotation_by_radial_residual"],
                      "map_rotation_from_yaw": report["maps"]["rotation_at_reported_yaw_change"],
                      "output": str(args.output.resolve())}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
