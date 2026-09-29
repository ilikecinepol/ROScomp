"""Локальный анализ короткого теста: журнал команд, телеметрия и JPEG.

Модуль ничего не отправляет роботу. Данные odom/SPORT названы сообщаемыми
оценками: их изменение не выдаётся за независимо измеренный путь. Камера
сравнивается в пикселях, без преобразования в метры или углы.
"""
from __future__ import annotations

import argparse
from collections import Counter
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from audit_saved_frames import camera_features, compare_camera, quaternion_rpy


def _read_jsonl(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _time(record):
    for key in ("elapsed_s", "t_s", "monotonic_s"):
        if key in record:
            value = float(record[key])
            if not math.isfinite(value):
                raise ValueError("Неконечное время приёма")
            return value
    raise ValueError("Нет elapsed_s, t_s или monotonic_s")


def _stamp(data):
    stamp = data.get("stamp", data.get("header", {}).get("stamp"))
    if isinstance(stamp, dict):
        return float(stamp["sec"])+float(stamp["nanosec"])*1e-9
    return None


def telemetry_rows(records, topic):
    rows = []
    for record in records:
        if record.get("topic") != topic:
            continue
        data = record["message"]["data"]
        row = {"t_s": _time(record), "phase": record.get("phase", "unknown"), "sensor_stamp_s": _stamp(data)}
        if topic == "ROBOTODOM":
            pose = data["pose"]
            row["position_m"] = [pose["position"][key] for key in "xyz"]
            row["rpy_rad"] = quaternion_rpy([pose["orientation"][key] for key in "xyzw"]).tolist()
            row["frame_id"] = data.get("header", {}).get("frame_id")
        else:
            row["rpy_rad"] = data["imu_state"]["rpy"]
            for key in ("position", "velocity", "yaw_speed", "mode", "gait_type", "error_code", "body_height", "foot_force"):
                if key in data:
                    row[{"position": "position_m", "velocity": "reported_velocity"}.get(key, key)] = data[key]
        rows.append(row)
    rows.sort(key=lambda row: row["t_s"])
    if rows:
        unwrapped = np.unwrap([row["rpy_rad"][2] for row in rows])
        for row, yaw in zip(rows, unwrapped):
            row["yaw_unwrapped_deg"] = float(np.degrees(yaw))
    return rows


def _phase_stats(rows):
    if not rows:
        return {"count": 0}
    receive = np.array([row["t_s"] for row in rows])
    yaw = np.array([row["yaw_unwrapped_deg"] for row in rows])
    stamps = [row["sensor_stamp_s"] for row in rows]
    use_sensor = all(value is not None for value in stamps) and len(set(stamps)) > 1
    times = np.asarray(stamps if use_sensor else receive, dtype=float)
    result = {"count": len(rows), "receive_first_s": float(receive[0]), "receive_last_s": float(receive[-1]),
              "receive_duration_s": float(np.ptp(receive)), "yaw_first_to_last_deg": float(yaw[-1]-yaw[0]),
              "yaw_min_deg": float(np.min(yaw)), "yaw_max_deg": float(np.max(yaw)),
              "yaw_mean_deg": float(np.mean(yaw)), "yaw_fit_time_source": "sensor_stamp" if use_sensor else "receive"}
    if len(rows) >= 3 and np.ptp(times) > .1:
        slope, intercept = np.polyfit(times-times[0], yaw, 1)
        residual = yaw-(slope*(times-times[0])+intercept)
        result.update({"yaw_rate_fit_deg_s": float(slope), "yaw_fit_rms_deg": float(np.sqrt(np.mean(residual**2)))})
    if all("position_m" in row for row in rows):
        positions = np.array([row["position_m"] for row in rows], dtype=float)
        result["reported_position_first_to_last_m"] = (positions[-1]-positions[0]).tolist()
        result["reported_position_axis_ranges_m"] = np.ptp(positions, axis=0).tolist()
    if all("reported_velocity" in row for row in rows):
        velocity = np.array([row["reported_velocity"] for row in rows], dtype=float)
        result.update({"reported_velocity_mean": np.mean(velocity, axis=0).tolist(),
                       "reported_velocity_abs_max": np.max(np.abs(velocity), axis=0).tolist(),
                       "reported_speed_norm_max": float(np.linalg.norm(velocity, axis=1).max()),
                       "reported_speed_norm_median": float(np.median(np.linalg.norm(velocity, axis=1)))})
    for key in ("mode", "gait_type", "error_code", "frame_id"):
        values = Counter(str(row[key]) for row in rows if key in row)
        if values:
            result[key+"_counts"] = dict(values)
    return result


def analyze_telemetry(rows, window=.5):
    phases = {phase: _phase_stats([row for row in rows if row["phase"] == phase]) for phase in dict.fromkeys(row["phase"] for row in rows)}
    before = [row for row in rows if row["phase"] == "before"]
    after = [row for row in rows if row["phase"] == "after"]
    result = {"phases": phases, "rows": rows, "physical_displacement_established": False}
    if not before or not after:
        result["before_after_status"] = "missing_before_or_after_phase"
        return result
    pre = [row for row in before if row["t_s"] >= before[-1]["t_s"]-window]
    post = [row for row in after if row["t_s"] >= after[-1]["t_s"]-window]
    yaw0, yaw1 = (float(np.median([row["yaw_unwrapped_deg"] for row in part])) for part in (pre, post))
    delta = {"window_definition": "Последние 0.5 с before и последние 0.5 с after по приёму",
             "before_n": len(pre), "after_n": len(post), "reported_yaw_change_deg": yaw1-yaw0,
             "before_window_t_s": [pre[0]["t_s"], pre[-1]["t_s"]],
             "after_window_t_s": [post[0]["t_s"], post[-1]["t_s"]]}
    if all("position_m" in row for row in pre+post):
        pos0, pos1 = (np.median([row["position_m"] for row in part], axis=0) for part in (pre, post))
        change = pos1-pos0
        angle = math.radians(yaw0)
        delta.update({"reported_position_delta_m": change.tolist(), "reported_position_delta_xy_norm_m": float(np.linalg.norm(change[:2])),
                      "projection_using_reported_before_yaw_xy_m": [float(change[0]*math.cos(angle)+change[1]*math.sin(angle)),
                                                                  float(-change[0]*math.sin(angle)+change[1]*math.cos(angle))],
                      "projection_limit": "Проекция по сообщаемому курсу, не проверенная система координат физического помещения"})
    if len(before) >= 10 and before[-1]["t_s"]-before[0]["t_s"] >= .5:
        t = np.array([row["t_s"] for row in before])
        y = np.array([row["yaw_unwrapped_deg"] for row in before])
        slope, intercept = np.polyfit(t-t[0], y, 1)
        elapsed = np.median([row["t_s"] for row in post])-np.median([row["t_s"] for row in pre])
        delta.update({"before_yaw_rate_by_receive_deg_s": float(slope),
                      "yaw_residual_vs_before_linear_extrapolation_deg": float(yaw1-yaw0-slope*elapsed),
                      "extrapolation_limit": "Описательный остаток от линейного тренда, не исправленный истинный курс"})
    result["before_after_reported_delta"] = delta
    return result


def _response_codes(value, path="response"):
    """Сохраняет места полей code; не приравнивает код к выполнению движения."""
    found = []
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except (ValueError, TypeError):
            return found
    if isinstance(value, dict):
        for key, child in value.items():
            child_path = path+"."+key
            if key in ("code", "status_code", "error_code") and isinstance(child, (str, int, float)):
                found.append({"path": child_path, "value": child})
            elif isinstance(child, (dict, list)):
                found.extend(_response_codes(child, child_path))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            found.extend(_response_codes(child, path+f"[{index}]"))
    return found


def analyze_events(events):
    ledger = []
    for event in events:
        command = event.get("command", event.get("command_name"))
        text = json.dumps(command, ensure_ascii=False).lower() if command is not None else ""
        tag = str(event.get("event", "")).lower()
        semantic = "StopMove" if "stopmove" in text else "Move" if "move" in text else "unclassified"
        row = {"event": event.get("event"), "t_s": _time(event), "phase": event.get("phase"),
               "command": command, "command_semantic_from_explicit_name": semantic,
               "parameter": event.get("parameter"), "reason": event.get("reason"),
               "response_present": "response" in event, "response": event.get("response"),
               "response_codes_uninterpreted": _response_codes(event.get("response")),
               "raw_event": event}
        row["request_evidence"] = command is not None and not any(word in tag for word in ("ack", "response", "result", "error"))
        response = event.get("response")
        try:
            status_code = response["data"]["header"]["status"]["code"]
        except (KeyError, TypeError):
            status_code = None
        row["unitree_header_status_code"] = status_code
        row["protocol_status_zero"] = status_code == 0 if status_code is not None else None
        ledger.append(row)
    ledger.sort(key=lambda row: row["t_s"])
    moves = [row["t_s"] for row in ledger if row["request_evidence"] and row["command_semantic_from_explicit_name"] == "Move"]
    stops = [row for row in ledger if row["request_evidence"] and row["command_semantic_from_explicit_name"] == "StopMove"]
    later = [row for row in stops if not moves or row["t_s"] >= max(moves)]
    stop_responses = [row for row in ledger if row["response_present"] and row["command_semantic_from_explicit_name"] == "StopMove"]
    move_responses = [row for row in ledger if row["response_present"] and row["command_semantic_from_explicit_name"] == "Move"]
    pending, matches, unmatched = {}, [], []
    for index, row in enumerate(ledger):
        semantic = row["command_semantic_from_explicit_name"]
        if row["request_evidence"]:
            pending.setdefault(semantic, []).append((index, row))
        elif row["response_present"] or row["event"] == "command_error":
            if pending.get(semantic):
                request_index, request = pending[semantic].pop(0)
                matches.append({"command": semantic, "request_event_index": request_index, "outcome_event_index": index,
                                "request_to_outcome_s": row["t_s"]-request["t_s"], "outcome_event": row["event"],
                                "status_code": row["unitree_header_status_code"],
                                "pairing": "По очереди запросов одинакового имени; уникальные request ID в схеме не заданы"})
            else:
                unmatched.append(index)
    return {"ledger": ledger, "move_request_event_count": len(moves), "stopmove_request_event_count": len(stops),
            "move_response_event_count": len(move_responses),
            "move_protocol_status_zero_count": sum(row["protocol_status_zero"] is True for row in move_responses),
            "stopmove_response_event_count": len(stop_responses),
            "stopmove_protocol_status_zero_count": sum(row["protocol_status_zero"] is True for row in stop_responses),
            "request_outcome_pairs": matches, "response_or_error_without_logged_request_indices": unmatched,
            "request_without_logged_outcome_indices": [index for queue in pending.values() for index, row in queue],
            "last_move_request_t_s": max(moves) if moves else None,
            "first_move_request_t_s": min(moves) if moves else None,
            "first_stopmove_request_after_last_move_t_s": later[0]["t_s"] if later else None,
            "first_move_to_first_subsequent_stop_request_s": later[0]["t_s"]-min(moves) if moves and later else None,
            "stopmove_request_after_last_move_recorded": bool(later),
            "stopmove_protocol_status_zero_after_last_move_recorded": any(row["protocol_status_zero"] is True and (not moves or row["t_s"] >= max(moves)) for row in stop_responses),
            "physical_stop_established_by_ack": False,
            "interpretation": "Запрос, ответ и физическое действие различаются. Числовые API ID без явного имени не интерпретируются автоматически."}


def analyze_camera(capture, summary):
    records = summary.get("camera_records", [])
    if not records:
        return {"status": "no_camera_records", "frames": [], "pixels_to_angles_calibrated": False}
    records = sorted(records, key=_time)
    before = [index for index, row in enumerate(records) if row.get("phase") == "before"]
    reference_index = before[-1] if before else 0
    warnings = [] if before else ["Нет phase=before у кадров: сравнение ведётся с первым сохранённым изображением"]
    features = []
    for record in records:
        path = (capture/record["file"]).resolve()
        if not path.is_relative_to(capture.resolve()):
            raise ValueError("Путь камеры выходит за папку записи")
        features.append(camera_features(path))
    rows = []
    for index, record in enumerate(records):
        row = dict(record)
        row["t_s"] = _time(record)
        row["comparison_to_reference"] = compare_camera(features[reference_index], features[index])
        row["comparison_to_previous"] = compare_camera(features[index-1], features[index]) if index else None
        if record.get("pts") is not None and record.get("time_base"):
            row["pts_seconds_declared"] = float(int(record["pts"])*Fraction(record["time_base"]))
        rows.append(row)
    pts = [row for row in rows if "pts_seconds_declared" in row]
    timing = None
    if len(pts) > 1:
        timing = {"receive_span_s": pts[-1]["t_s"]-pts[0]["t_s"],
                  "declared_pts_span_s": pts[-1]["pts_seconds_declared"]-pts[0]["pts_seconds_declared"],
                  "sensor_synchronization_verified": False}
    return {"status": "processed", "reference_file": records[reference_index]["file"],
            "reference_phase": records[reference_index].get("phase", "unknown"), "warnings": warnings,
            "frames": rows, "timing": timing, "pixels_to_angles_calibrated": False,
            "interpretation": "Регистрация изображения в пикселях. Не устанавливает путь в метрах, точный угол поворота, свежесть экспозиции или физическую остановку."}


def write_markdown(report, path):
    lines = ["# Анализ короткого теста движения Go2", "",
             "Это анализ сохранённого журнала. Изменения odom/SPORT — сообщаемые оценки, а не независимое измерение физического пути. Ответ команды не доказывает её физическое выполнение.", "",
             f"Запись: `{report['capture']}`. Сохранённых событий: {len(report['events']['ledger'])}.", "",
             "## Запросы и ответы", "",
             f"Явно названных запросов Move: {report['events']['move_request_event_count']}; StopMove: {report['events']['stopmove_request_event_count']}. StopMove после последнего Move найден: {report['events']['stopmove_request_after_last_move_recorded']}.", "",
             f"Ответов Move с header.status.code=0: {report['events']['move_protocol_status_zero_count']}; StopMove: {report['events']['stopmove_protocol_status_zero_count']}. Это подтверждение протокола, а не измерение результата действия.", "",
             "Полная последовательность, ответы и поля кодов без интерпретации сохранены в JSON. Если имена команд отсутствуют, числовые API ID не подменяются догадками.", "",
             "## Телеметрия по фазам", ""]
    for topic, analysis in report["telemetry"].items():
        lines.append(f"### {topic}")
        lines.append("")
        for phase, stats in analysis["phases"].items():
            line = f"- {phase}: {stats['count']} сообщений, {stats['receive_duration_s']:.3f} с; изменение yaw {stats['yaw_first_to_last_deg']:.4f}°."
            if "reported_speed_norm_max" in stats:
                line += f" Максимальная норма сообщаемой скорости {stats['reported_speed_norm_max']:.5f}."
            lines.append(line)
        delta = analysis.get("before_after_reported_delta")
        if delta:
            lines += ["", f"Окна последних 0,5 с before/after: изменение yaw {delta['reported_yaw_change_deg']:.5f}°."]
            if "reported_position_delta_m" in delta:
                lines.append(f"Изменение сообщаемой позиции XYZ: {delta['reported_position_delta_m']} м; норма XY {delta['reported_position_delta_xy_norm_m']:.6f} м.")
            if "yaw_residual_vs_before_linear_extrapolation_deg" in delta:
                lines.append(f"Остаток yaw относительно линейного продолжения тренда before: {delta['yaw_residual_vs_before_linear_extrapolation_deg']:.5f}°; это не исправление истинного курса.")
        else:
            lines += ["", "До/после не сравнивается: отсутствует одна из необходимых фаз."]
        lines.append("")
    lines += ["## Изображения", ""]
    camera = report["camera"]
    if camera["frames"]:
        lines.append(f"Опорный кадр: {camera['reference_file']}, фаза {camera['reference_phase']}.")
        for row in camera["frames"]:
            metrics = row["comparison_to_reference"]
            displacement = metrics.get("inlier_p95_displacement_norm_px")
            if displacement is not None:
                lines.append(f"- {row['file']} ({row.get('phase', 'unknown')}): согласованных признаков {metrics['ransac_inliers']}, P95 смещения {displacement:.4f} px при 640×360.")
            else:
                lines.append(f"- {row['file']}: надёжная гомография не получена.")
        lines += ["", "Метрики даны в пикселях; перевод в физический угол или расстояние не выполнялся. Время приёма не является временем экспозиции."]
    else:
        lines.append("Нет сохранённых кадров с метаданными.")
    lines += ["", "## Ограничения", "", *[f"- {warning}" for warning in report["limitations"]], ""]
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, default=Path("outputs/go2-motion-test/analysis"))
    args = parser.parse_args()
    cv2.setNumThreads(1)
    cv2.setRNGSeed(0)
    summary = json.loads((args.capture/"summary.json").read_text(encoding="utf-8"))
    samples = _read_jsonl(args.capture/"samples.jsonl")
    events = _read_jsonl(args.capture/"events.jsonl")
    telemetry = {topic: analyze_telemetry(telemetry_rows(samples, topic)) for topic in ("ROBOTODOM", "LF_SPORT_MOD_STATE", "LOW_STATE")}
    report = {"schema_version": 1, "capture": args.capture.name, "analysis_only_no_robot_connection": True,
              "telemetry": telemetry, "events": analyze_events(events), "camera": analyze_camera(args.capture, summary),
              "capture_summary_errors": summary.get("errors", []), "physical_displacement_established": False,
              "capture_plan": summary.get("plan"), "capture_executed_flag": summary.get("executed"),
              "capture_pulse_summary": summary.get("pulse"), "capture_final_stop_acknowledged_flag": summary.get("final_stop_acknowledged"),
              "limitations": ["Системы координат и масштаб odom физически не подтверждены",
                              "Данные скорости и позиции не являются независимым измерением пройденного пути",
                              "ACK и коды ответа не доказывают движение или остановку",
                              "Камера анализируется в пикселях без калибровки и синхронизации экспозиции",
                              "Времена elapsed_s/t_s принимаются как монотонная шкала источника; разные начала отсчёта нельзя смешивать"],
              "input_sha256": {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(args.capture.iterdir())
                               if path.suffix in (".json", ".jsonl", ".jpg")},
              "analysis_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/"motion-analysis.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    write_markdown(report, args.output/"АНАЛИЗ ТЕСТА ДВИЖЕНИЯ.md")
    print(json.dumps({"capture": report["capture"], "telemetry": {topic: {key: value for key, value in result.items() if key != 'rows'} for topic, result in telemetry.items()},
                      "events": {key: value for key, value in report['events'].items() if key != 'ledger'},
                      "output": str(args.output.resolve())}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
