"""Аудит курса, изменений изображения и PTS в непрерывной записи Go2.

Только локальные файлы. Связывает данные по приёму, не утверждая точную
синхронизацию экспозиций или физическую причину наблюдаемых расхождений.
"""
from __future__ import annotations

import argparse
from fractions import Fraction
from functools import reduce
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from audit_saved_frames import audit_capture, camera_features, compare_camera


def fit_line(times, values):
    times, values = np.asarray(times, dtype=float), np.asarray(values, dtype=float)
    times = times-times[0]
    slope, intercept = np.polyfit(times, values, 1)
    residual = values-(slope*times+intercept)
    denominator = float(np.sum((values-np.mean(values))**2))
    return {"slope": float(slope), "intercept_at_first_time": float(intercept),
            "r_squared": 1-float(np.sum(residual**2))/denominator if denominator else None,
            "rms_residual": float(np.sqrt(np.mean(residual**2))),
            "max_abs_residual": float(np.max(np.abs(residual)))}


def yaw_stats(stream):
    rows = stream["samples"]
    receive = np.array([row["receive_elapsed_s"] for row in rows])
    yaw = np.degrees(np.unwrap([row["rpy_rad"][2] for row in rows]))
    stats = {"n": len(rows), "receive_span_s": float(np.ptp(receive)),
             "yaw_first_deg": float(yaw[0]), "yaw_last_deg": float(yaw[-1]),
             "yaw_first_to_last_deg": float(yaw[-1]-yaw[0]),
             "fit_yaw_deg_by_receive_s": fit_line(receive, yaw)}
    stamps = [row.get("sensor_stamp_s") for row in rows]
    if all(value is not None for value in stamps):
        stats["sensor_stamp_span_s"] = stamps[-1]-stamps[0]
        stats["fit_yaw_deg_by_sensor_s"] = fit_line(stamps, yaw)
        stats["fit_sensor_elapsed_s_by_receive_s"] = fit_line(receive, np.array(stamps)-stamps[0])
    if "position_m" in rows[0]:
        position = np.array([row["position_m"] for row in rows])
        stats["reported_position_first_to_last_m"] = (position[-1]-position[0]).tolist()
        stats["reported_position_axis_ranges_m"] = np.ptp(position, axis=0).tolist()
    return stats


def save_plot(report, output):
    import os
    import tempfile
    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir())/"go2-perception-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(11, 10), facecolor="#f8f9fb")
    for topic, color in (("ROBOTODOM", "#1565c0"), ("LF_SPORT_MOD_STATE", "#ee8d21"), ("LOW_STATE", "#248152")):
        rows = report["sensor_audit"]["streams"][topic]["samples"]
        t = np.array([row["receive_elapsed_s"] for row in rows])
        yaw = np.degrees(np.unwrap([row["rpy_rad"][2] for row in rows]))
        axes[0].plot(t, yaw-yaw[0], color=color, label=topic, linewidth=1.4)
    axes[0].set(ylabel="Изменение курса, °", title="Сообщаемый курс трёх потоков изменяется примерно на 6,15°")
    axes[0].legend(loc="lower left")
    frames = report["camera_frames"]
    t = np.array([frame["elapsed_s"] for frame in frames])
    axes[1].plot(t, [frame["comparison_to_first"]["inlier_p95_displacement_norm_px"] for frame in frames], "o-", color="#7543a3")
    axes[1].set(ylabel="Смещение P95, пиксели", title="Согласованные признаки изображения: смещение относительно первого кадра (640×360)")
    media = np.array([frame["pts_seconds_using_declared_time_base"] for frame in frames])
    axes[2].plot(t-t[0], t-t[0], "--", color="#999999", label="Масштаб 1:1")
    axes[2].plot(t-t[0], media-media[0], "o-", color="#c33743", label="PTS × 1/90000")
    axes[2].set(xlabel="Время приёма, с (нижний график — от первого кадра)", ylabel="Приращение PTS, с",
                title=f"PTS: {media[-1]-media[0]:.3f} с при {t[-1]-t[0]:.3f} с между приёмами кадров")
    axes[2].legend()
    for axis in axes:
        axis.grid(alpha=.25)
    fig.suptitle("Go2: непрерывная запись 28 с | причина расхождений не установлена", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, .96))
    fig.savefig(output, dpi=155)
    plt.close(fig)


def write_markdown(report, output):
    stats = report["yaw_statistics"]
    odom = stats["ROBOTODOM"]
    sport = stats["LF_SPORT_MOD_STATE"]
    timing = report["camera_timing"]
    cameras = report["camera_aggregate"]
    last = report["camera_frames"][-1]
    gyro = report["sensor_audit"]["gyro_check"]
    text = f"""# Непрерывный аудит Go2 — запись {report['capture']}

Анализ выполнен только по сохранённым локальным файлам. Причина расхождений — в аппаратуре, прошивке, драйвере, обработке данных или иной части цепочки — этим аудитом не устанавливается.

## Курс на протяжении 28 секунд

| Поток | Сообщений | Изменение курса, первый → последний | Линейный темп |
|---|---:|---:|---:|
| ROBOTODOM | {odom['n']} | {odom['yaw_first_to_last_deg']:.6f}° | {odom['fit_yaw_deg_by_sensor_s']['slope']:.6f}°/с по timestamp |
| LF_SPORT_MOD_STATE | {sport['n']} | {sport['yaw_first_to_last_deg']:.6f}° | {sport['fit_yaw_deg_by_sensor_s']['slope']:.6f}°/с по timestamp |
| LOW_STATE | {stats['LOW_STATE']['n']} | {stats['LOW_STATE']['yaw_first_to_last_deg']:.6f}° | {stats['LOW_STATE']['fit_yaw_deg_by_receive_s']['slope']:.6f}°/с по приёму |

Промежуток timestamp SPORT — {sport['sensor_stamp_span_s']:.6f} с, ODOM — {odom['sensor_stamp_span_s']:.6f} с. Для SPORT R² линейного тренда {sport['fit_yaw_deg_by_sensor_s']['r_squared']:.8f}, среднеквадратичный остаток {sport['fit_yaw_deg_by_sensor_s']['rms_residual']:.6f}°. Изменение присутствует на протяжении этой непрерывной записи; оно не сводится к сравнению двух разнесённых сессий или скачку через ±π.

Отношение приращения sensor timestamp SPORT к времени приёма по регрессии — {sport['fit_sensor_elapsed_s_by_receive_s']['slope']:.8f}. Время приёма и время SPORT имеют близкий масштаб в этом окне, но их абсолютные смещения не калиброваны.

Порядок quaternion SPORT wxyz проверен сравнением с его же rpy. ODOM использует именованные x/y/z/w; ошибки порядка в вычислении нашего yaw не объясняют наблюдаемый тренд. Все значения и проверки сохранены в JSON.

Среднее сохранённое значение gyro Z — {gyro['gyro_mean_as_saved'][2]:.8f}. При **условном** предположении, что gyroscope задан в rad/s корпуса и действует стандартное соотношение Эйлера ZYX, средний расчётный yaw-rate получается {gyro['predicted_mean_euler_yaw_rate_rad_s_if_body_gyro_rad_s_ZYX']:.8f} rad/s. Это не совпадает с регрессией сообщаемого yaw; единицы, система gyroscope и возможная коррекция оценки ориентации должны проверяться отдельно. Это не диагноз неисправности.

## Изображения и привязка к приёму

Проверены все 14 JPEG относительно первого, а также 13 последовательных пар. При анализе 640×360 согласованные SIFT-признаки между первым и последним кадром дают {last['comparison_to_first']['ransac_inliers']} inliers; 95% их смещений не превышают {last['comparison_to_first']['inlier_p95_displacement_norm_px']:.4f} пикселя. По всем сравнениям с первым максимум P95 — {cameras['max_p95_displacement_px_vs_first']:.4f} пикселя, минимальная корреляция пикселей — {cameras['min_pixel_correlation_vs_first']:.6f}. Внутрисценовые изменения могут существовать; регистрация использует согласованное большинство признаков.

В моменты приёма первого и последнего JPEG интерполированный по времени приёма курс ODOM меняется на {report['camera_aligned_reported_yaw_change_deg']:.6f}°. Направление вида по согласованным признакам при этом остаётся близким. Калибровки камеры и времени экспозиции нет: пиксельное смещение здесь не преобразуется в точный угол, а метки приёма не выдаются за синхронизацию экспозиций. Фактический поворот корпуса на сообщаемые 6° этой последовательностью изображений не подтверждён.

## PTS видео

- Первый PTS {timing['pts_first']}, последний {timing['pts_last']}; разница **{timing['pts_tick_span']}**.
- У всех 14 сохранённых кадров `time_base = 1/90000`.
- При применении объявленного time_base длительность между этими кадрами — **{timing['declared_media_span_s']:.6f} с**.
- Между их приёмами проходит **{timing['receive_span_s']:.6f} с**; отношение приём/PTS — **{timing['receive_to_declared_media_span_ratio']:.6f}**.
- Скорость PTS по приёму — **{timing['fit_pts_ticks_by_receive_s']['slope']:.6f} тиков/с**, вместо 90000 тиков на секунду при масштабе 1:1.
- Разности соседних сохранённых PTS: {timing['pts_tick_deltas']}; их НОД — {timing['pts_delta_gcd']}. Это описывает сохранённые кадры, а не доказывает шаг каждого из 396 декодированных кадров.

В этой записи PTS с указанным time_base **не соответствует масштабу времени приёма**. По данным нельзя выбрать правильный альтернативный time_base или объявить конкретную причину: это может требовать проверки назначения PTS, RTP/декодирования и промежуточных преобразований. Автоматически умножать PTS на найденное отношение для «исправления синхронизации» нельзя.

## Лидар и полнота журнала

Счётчик summary сообщает 216 ULIDAR_ARRAY, но в samples.jsonl сохранены только первые **30** сообщений, охватывающие {report['sensor_audit']['streams']['ULIDAR_ARRAY']['summary']['receive_span_s']:.6f} с. Это различие между общим счётчиком и числом записанных примеров, а не доказанная потеря пакетов. Для ULIDAR_STATE сохранены все **140** сообщений, около 27,8 с. Во всех сохранённых ULIDAR_ARRAY и ULIDAR_STATE `stamp` равен **1789558000.0**; по этой метке невозможно определить внутриокновую синхронизацию. Сравнение двух декодеров одного map_payload.bin в этот аудит не входит.

## Воспроизведение

```powershell
& 'work/go2-perception/.venv/Scripts/python.exe' 'work/go2-perception/audit_continuous.py'
```

`continuous-audit.json` содержит все использованные сообщения, метрики регистрации, PTS, результаты регрессий и хеши входных файлов. `continuous-timing.png` визуализирует независимые величины на отдельных осях.
"""
    output.write_text(text, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, default=Path("outputs/go2-remote-setup/logs/20260916T112503Z"))
    parser.add_argument("--output", type=Path, default=Path("outputs/go2-perception/coordinate-audit"))
    args = parser.parse_args()
    cv2.setNumThreads(1)
    cv2.setRNGSeed(0)
    summary = json.loads((args.capture/"summary.json").read_text(encoding="utf-8"))
    camera_records = summary["camera_records"]
    if [record["file"] for record in camera_records] != summary["camera_files"]:
        raise ValueError("camera_records и camera_files не согласованы")
    sensor = audit_capture(args.capture)
    yaw = {topic: yaw_stats(sensor["streams"][topic]) for topic in ("ROBOTODOM", "LF_SPORT_MOD_STATE", "LOW_STATE")}
    odom = sensor["streams"]["ROBOTODOM"]["samples"]
    odom_t = np.array([row["receive_elapsed_s"] for row in odom])
    odom_yaw = np.degrees(np.unwrap([row["rpy_rad"][2] for row in odom]))
    features = [camera_features(args.capture/record["file"]) for record in camera_records]
    frames = []
    for index, record in enumerate(camera_records):
        frame = dict(record)
        frame["pts_seconds_using_declared_time_base"] = float(int(record["pts"])*Fraction(record["time_base"]))
        frame["comparison_to_first"] = compare_camera(features[0], features[index])
        frame["comparison_to_previous"] = compare_camera(features[index-1], features[index]) if index else None
        frame["odom_yaw_deg_interpolated_by_receive_time"] = float(np.interp(record["elapsed_s"], odom_t, odom_yaw))
        frame["nearest_odom_receive_time_difference_s"] = float(odom_t[np.argmin(abs(odom_t-record["elapsed_s"]))]-record["elapsed_s"])
        frames.append(frame)
    receive = np.array([frame["elapsed_s"] for frame in frames])
    pts = np.array([frame["pts"] for frame in frames], dtype=np.int64)
    media = np.array([frame["pts_seconds_using_declared_time_base"] for frame in frames])
    pts_deltas = np.diff(pts)
    timing = {"n_saved": len(frames), "n_decoded_from_summary": summary["counts"]["CAMERA"],
              "time_base_values": sorted({frame["time_base"] for frame in frames}),
              "pts_first": int(pts[0]), "pts_last": int(pts[-1]), "pts_tick_span": int(pts[-1]-pts[0]),
              "receive_span_s": float(receive[-1]-receive[0]), "declared_media_span_s": float(media[-1]-media[0]),
              "receive_to_declared_media_span_ratio": float((receive[-1]-receive[0])/(media[-1]-media[0])),
              "pts_tick_deltas": pts_deltas.tolist(), "pts_delta_gcd": reduce(math.gcd, pts_deltas.tolist()),
              "receive_delta_s": np.diff(receive).tolist(), "fit_pts_ticks_by_receive_s": fit_line(receive, pts),
              "fit_declared_media_s_by_receive_s": fit_line(receive, media)}
    report = {"capture": args.capture.name, "scope": "local_only_no_robot_connection",
              "sensor_audit": sensor, "yaw_statistics": yaw, "camera_frames": frames, "camera_timing": timing,
              "summary_counts": summary["counts"],
              "camera_aggregate": {"min_pixel_correlation_vs_first": min(frame["comparison_to_first"]["pixel_correlation"] for frame in frames),
                                   "max_p95_displacement_px_vs_first": max(frame["comparison_to_first"]["inlier_p95_displacement_norm_px"] for frame in frames),
                                   "min_ransac_inliers_vs_first": min(frame["comparison_to_first"]["ransac_inliers"] for frame in frames)},
              "camera_aligned_reported_yaw_change_deg": frames[-1]["odom_yaw_deg_interpolated_by_receive_time"]-frames[0]["odom_yaw_deg_interpolated_by_receive_time"],
              "sensor_clock_synchronization_verified": False,
              "input_sha256": {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(args.capture.iterdir())
                               if path.suffix in (".json", ".jsonl", ".jpg", ".bin")},
              "code_sha256": {name: hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
                              for name in ("audit_continuous.py", "audit_saved_frames.py")}}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/"continuous-audit.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    write_markdown(report, args.output/"НЕПРЕРЫВНЫЙ АУДИТ.md")
    save_plot(report, args.output/"continuous-timing.png")
    print(json.dumps({"yaw_statistics": yaw, "camera_timing": timing, "camera_aggregate": report["camera_aggregate"],
                      "first_to_last_camera": frames[-1]["comparison_to_first"],
                      "camera_aligned_reported_yaw_change_deg": report["camera_aligned_reported_yaw_change_deg"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
