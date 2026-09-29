"""Локальный геометрический поиск серой рампы с поперечными планками.

Это исследовательский детектор, а не команда движения и не классификатор с
измеренной вероятностью. Он предполагает камеру примерно без крена, рампу,
видимую целиком спереди, и не менее двух внутренних поперечных планок.
Калибровка отсутствует: все координаты и направления относятся только к кадру.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


def _line(p):
    p = np.asarray(p, dtype=np.float64).reshape(2, 2)
    if p[0, 0] > p[1, 0]:
        p = p[::-1]
    return p


def _line_y(p, x):
    return float(p[0, 1] + (x - p[0, 0]) * (p[1, 1] - p[0, 1]) / max(p[1, 0] - p[0, 0], 1e-6))


def _quad_width_at_y(quad, y):
    left = quad[[0, 3]]
    right = quad[[1, 2]]
    def x_at(edge):
        t = (y - edge[0, 1]) / (edge[1, 1] - edge[0, 1] + 1e-9)
        return float(edge[0, 0] + t * (edge[1, 0] - edge[0, 0]))
    return x_at(left), x_at(right)


def _extract_lines(gray):
    height, width = gray.shape
    found = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD).detect(gray)[0]
    candidates = []
    if found is None:
        return candidates
    for raw in found.reshape(-1, 4):
        p = _line(raw)
        dx, dy = p[1] - p[0]
        if dx < max(18.0, width * .022) or abs(dy) > dx * .32:
            continue
        candidates.append(p)
    # Одна граница может распасться при смене фона; сшиваем только близкие
    # коллинеарные отрезки, не используя положение объекта в кадре.
    for _ in range(64):
        merged = False
        for i, p in enumerate(candidates):
            if merged:
                break
            for j in range(i + 1, len(candidates)):
                q = candidates[j]
                gap = max(p[0, 0], q[0, 0]) - min(p[1, 0], q[1, 0])
                if not -3. <= gap <= max(6., width * .015):
                    continue
                slope_p = (p[1, 1] - p[0, 1]) / np.ptp(p[:, 0])
                slope_q = (q[1, 1] - q[0, 1]) / np.ptp(q[:, 0])
                mid_x = (max(p[0, 0], q[0, 0]) + min(p[1, 0], q[1, 0])) / 2
                # Широкоугольная камера изгибает длинную поперечину:
                # соседние короткие касательные могут различаться наклоном.
                if abs(slope_p - slope_q) < .10 and abs(_line_y(p, mid_x) - _line_y(q, mid_x)) < 4.5:
                    points = np.concatenate((p, q))
                    candidates[i] = points[[np.argmin(points[:, 0]), np.argmax(points[:, 0])]]
                    candidates.pop(j)
                    merged = True
                    break
        if not merged:
            break
    # У поперечной планки есть две близкие границы; это один уровень.
    candidates.sort(key=lambda p: -np.linalg.norm(p[1] - p[0]))
    unique = []
    for p in candidates:
        duplicate = False
        for q in unique:
            overlap = min(p[1, 0], q[1, 0]) - max(p[0, 0], q[0, 0])
            if overlap < .7 * min(np.ptp(p[:, 0]), np.ptp(q[:, 0])):
                continue
            x = (max(p[0, 0], q[0, 0]) + min(p[1, 0], q[1, 0])) / 2
            if abs(_line_y(p, x) - _line_y(q, x)) < max(3., height * .015):
                duplicate = True
                break
        if not duplicate:
            unique.append(p)
    return sorted(unique, key=lambda p: float(p[:, 1].mean()))


def detect_ramp(image_bgr):
    """Возвращает JSON-совместимые гипотезы; confidence_heuristic не вероятность.

    Порядок четырёх углов: дальний левый, дальний правый, ближний правый,
    ближний левый. Ближней считается нижняя в изображении грань — это
    предположение о позе камеры, а не установленный по глубине факт.
    """
    if not isinstance(image_bgr, np.ndarray) or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError("Ожидается изображение BGR с тремя каналами")
    if image_bgr.dtype != np.uint8 or min(image_bgr.shape[:2]) < 32:
        raise ValueError("Ожидается uint8-кадр размером не менее 32 × 32")
    original_h, original_w = image_bgr.shape[:2]
    scale = min(1., 960. / original_w)
    frame = cv2.resize(image_bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1 else image_bgr
    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 30, 75)
    edge_distance = cv2.distanceTransform(255 - edges, cv2.DIST_L2, 3)
    lines = _extract_lines(gray)
    scene_value = float(np.median(hsv[:, :, 2]))
    hypotheses = []
    tested = 0
    rejected = {"geometry": 0, "transverse_support": 0, "surface": 0, "side_boundaries": 0}
    for top_index, top in enumerate(lines):
        for bottom in lines[top_index + 1:]:
            top_y, bottom_y = float(top[:, 1].mean()), float(bottom[:, 1].mean())
            depth = bottom_y - top_y
            top_w, bottom_w = np.ptp(top[:, 0]), np.ptp(bottom[:, 0])
            average_w = (top_w + bottom_w) * .5
            if depth < max(height * .035, average_w * .45) or depth > average_w * 3.2:
                continue
            if not .48 <= bottom_w / top_w <= 2.3:
                continue
            if abs(float(top[:, 0].mean() - bottom[:, 0].mean())) > depth * 1.4:
                continue
            tested += 1
            quad = np.array([top[0], top[1], bottom[1], bottom[0]], dtype=np.float32)
            area = abs(float(cv2.contourArea(quad)))
            if area < height * width * .004 or area > height * width * .45 or not cv2.isContourConvex(quad):
                rejected["geometry"] += 1
                continue
            # Проверяем повторяющиеся поперечины, доходящие до обеих сторон.
            support = []
            errors = []
            slope = ((top[1, 1] - top[0, 1]) / top_w + (bottom[1, 1] - bottom[0, 1]) / bottom_w) * .5
            for segment in lines:
                y = float(segment[:, 1].mean())
                t = (y - top_y) / depth
                if not .09 < t < .91:
                    continue
                left, right = _quad_width_at_y(quad, y)
                projected_width = right - left
                if projected_width <= 0:
                    continue
                endpoint_error = (abs(segment[0, 0] - left) + abs(segment[1, 0] - right)) / (2 * projected_width)
                segment_slope = (segment[1, 1] - segment[0, 1]) / np.ptp(segment[:, 0])
                if endpoint_error < .12 and abs(segment_slope - slope) < .12:
                    support.append(segment)
                    errors.append(endpoint_error)
            if len(support) < 2:
                rejected["transverse_support"] += 1
                continue
            side_support = []
            for start, end in ((quad[0], quad[3]), (quad[1], quad[2])):
                samples = start + np.linspace(.06, .94, 80)[:, None] * (end - start)
                xy = np.rint(samples).astype(np.int32)
                xy[:, 0] = np.clip(xy[:, 0], 0, width - 1)
                xy[:, 1] = np.clip(xy[:, 1], 0, height - 1)
                # Допуск кривизны линзы в пикселях; это не метрический зазор.
                side_support.append(float(np.mean(edge_distance[xy[:, 1], xy[:, 0]] <= max(3.5, width*.008))))
            if min(side_support) < .55:
                rejected["side_boundaries"] += 1
                continue
            # Цвет применяется ко всей гипотезе, а не к заранее заданной зоне.
            mask = np.zeros((height, width), np.uint8)
            cv2.fillConvexPoly(mask, np.rint(quad).astype(np.int32), 255)
            inner = cv2.erode(mask, np.ones((5, 5), np.uint8)) > 0
            pixels = hsv[inner]
            if len(pixels) < 50:
                continue
            median_saturation = float(np.median(pixels[:, 1]))
            median_value = float(np.median(pixels[:, 2]))
            neutral_fraction = float(np.mean(pixels[:, 1] <= 72))
            dark_ratio = median_value / max(scene_value, 1.)
            value_q10, value_q90 = np.percentile(pixels[:, 2], [10, 90])
            value_spread = float((value_q90 - value_q10) / max(median_value, 1.))
            # Тёмный серый цвет помогает исключать светлые стены и потолок;
            # несколько поперечин и форма исключают простые тонкие маты.
            if neutral_fraction < .65 or median_saturation > 65 or not 25 <= median_value <= 210 or dark_ratio > .85 or value_spread > .7:
                rejected["surface"] += 1
                continue
            support_error = float(np.mean(errors))
            score = float(np.clip(.35 + .10 * min(len(support), 4) + .2 * neutral_fraction + .15 * max(0., 1 - support_error / .12), 0., 1.))
            hypotheses.append({"quad": quad, "score": score, "support": support,
                "metrics": {"internal_transverse_levels": len(support), "mean_endpoint_error_fraction": round(support_error, 4),
                    "left_right_edge_support_fraction": [round(v, 4) for v in side_support],
                    "neutral_surface_fraction": round(neutral_fraction, 4), "median_saturation_0_255": median_saturation,
                    "median_value_0_255": median_value, "surface_to_scene_value_ratio": round(dark_ratio, 4),
                    "relative_value_spread_q10_q90": round(value_spread, 4),
                    "projected_depth_to_width_ratio": round(float(depth / average_w), 4)}})
    # Перекрывающиеся гипотезы одной рампы объединяются выбором более полной.
    hypotheses.sort(key=lambda c: (c["metrics"]["internal_transverse_levels"], c["score"], cv2.contourArea(c["quad"])), reverse=True)
    selected = []
    for item in hypotheses:
        duplicate = False
        for other in selected:
            intersection, _ = cv2.intersectConvexConvex(item["quad"], other["quad"])
            smaller_area = min(abs(cv2.contourArea(item["quad"])), abs(cv2.contourArea(other["quad"])))
            if intersection / max(smaller_area, 1.) > .55:
                duplicate = True
                break
        if not duplicate:
            selected.append(item)
        if len(selected) >= 5:
            break
    candidates = []
    for index, item in enumerate(selected):
        quad = item["quad"].astype(float) / scale
        entrance = quad[[3, 2]]
        center = entrance.mean(axis=0)
        far = quad[:2].mean(axis=0)
        axis = far - center
        axis /= max(float(np.linalg.norm(axis)), 1e-9)
        candidates.append({
            "id": index, "kind": "ramp_with_transverse_bars_candidate",
            "quadrilateral_px": np.round(quad, 2).tolist(),
            "quadrilateral_order": ["far_left", "far_right", "near_right", "near_left"],
            "entrance": {"endpoints_px": np.round(entrance, 2).tolist(), "center_px": np.round(center, 2).tolist(),
                "status": "observed_patch_boundary_not_verified_entrance", "verified": False},
            "motion_authorized": False,
            "full_ramp_verified": False,
            "direction": {"coordinate_frame": "image_nonmetric", "axis_unit_xy": np.round(axis, 5).tolist(),
                "axis_angle_from_image_up_deg": round(math.degrees(math.atan2(float(axis[0]), float(-axis[1]))), 3),
                "entrance_horizontal_offset_normalized": round(float((center[0] - original_w / 2) / (original_w / 2)), 5),
                "robot_bearing_deg": None, "distance_m": None},
            "confidence_heuristic": round(item["score"], 4), "confidence_is_probability": False,
            "evidence": item["metrics"],
            "transverse_lines_px": [np.round(p / scale, 2).tolist() for p in item["support"]],
        })
    return {"schema_version": 1, "image_size": {"width": original_w, "height": original_h},
        "candidates": candidates,
        "diagnostics": {"horizontal_line_count": len(lines), "quadrilateral_hypotheses_tested": tested,
            "rejected": rejected, "accepted_before_overlap_filter": len(hypotheses),
            "scene_median_value_0_255": scene_value,
            "method": "parallel_transverse_lines_and_dark_neutral_surface",
            "calibrated": False,
            "limitations": ["Проверено только на небольшой локальной выборке, качество на других ракурсах неизвестно.",
                "Нижняя грань гипотезы может быть поперечной планкой; физический вход не подтверждён.",
                "Нужны минимум две видимые внутренние поперечины; гладкая рампа может быть пропущена.",
                "Направление на изображении не является курсом робота; расстояние и уклон не определяются."]}}


def render_overlay(frame, result):
    """Рисует найденные границы; возвращает новую BGR-копию кадра."""
    overlay = frame.copy()
    for candidate in result.get("candidates", []):
        quad = np.rint(candidate["quadrilateral_px"]).astype(np.int32)
        cv2.polylines(overlay, [quad], True, (0, 220, 255), 2, cv2.LINE_AA)
        for line in candidate.get("transverse_lines_px", []):
            points = np.rint(line).astype(np.int32)
            cv2.line(overlay, tuple(points[0]), tuple(points[1]), (255, 160, 0), 2, cv2.LINE_AA)
        entrance = np.rint(candidate["entrance"]["endpoints_px"]).astype(np.int32)
        center = tuple(np.rint(candidate["entrance"]["center_px"]).astype(np.int32))
        far = tuple(np.rint(quad[:2].mean(axis=0)).astype(np.int32))
        cv2.line(overlay, tuple(entrance[0]), tuple(entrance[1]), (50, 255, 80), 4, cv2.LINE_AA)
        cv2.circle(overlay, center, 6, (50, 255, 80), -1, cv2.LINE_AA)
        cv2.arrowedLine(overlay, center, far, (50, 255, 80), 2, cv2.LINE_AA, tipLength=.12)
        # OpenCV не поддерживает кириллицу в стандартном шрифте; только числа.
        label = f"#{candidate['id']} {candidate['confidence_heuristic']:.2f}"
        cv2.putText(overlay, label, (int(quad[0, 0]), max(20, int(quad[0, 1]) - 10)), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 220, 255), 2, cv2.LINE_AA)
    return overlay


def main():
    parser = argparse.ArgumentParser(description="Поиск кандидата рампы на локальном изображении. Команд роботу нет.")
    parser.add_argument("input", type=Path, help="Входное изображение")
    parser.add_argument("--output", type=Path, required=True, help="Выходной JSON")
    parser.add_argument("--overlay", type=Path, help="Изображение с границами")
    args = parser.parse_args()
    frame = cv2.imdecode(np.fromfile(str(args.input), dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        parser.error("Не удалось прочитать изображение")
    result = detect_ramp(frame)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    if args.overlay:
        args.overlay.parent.mkdir(parents=True, exist_ok=True)
        extension = args.overlay.suffix or ".jpg"
        ok, encoded = cv2.imencode(extension, render_overlay(frame, result))
        if not ok:
            raise RuntimeError("Не удалось сохранить разметку")
        encoded.tofile(str(args.overlay))
    print(json.dumps({"candidates": len(result["candidates"]), "output": str(args.output)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
