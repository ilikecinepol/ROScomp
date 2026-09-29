"""Объяснение конфликтов карты с корпусом; не очищает занятые клетки."""
from dataclasses import replace
import math

import numpy as np

from .footprint import footprint_report


def body_map_audit(grid, top, floor_height, pose, policy, uncertainty):
    """Отделяет конфликт реального габарита от внешнего запаса.

    Высота помогает исследовать неверную привязку и отражения, но сама по себе
    не доказывает, что точку разрешено удалить как собственный корпус.
    """
    nominal_policy = replace(policy, clearance=1e-9)
    nominal = footprint_report(grid, pose, nominal_policy, 0.)
    expanded = footprint_report(grid, pose, policy, uncertainty)
    cells = np.asarray(grid.cells)
    top = np.asarray(top)
    floor_height = np.asarray(floor_height)
    if top.shape != cells.shape or floor_height.shape != cells.shape:
        raise ValueError('Размеры карты и высот не совпадают')
    co, si = math.cos(pose.yaw), math.sin(pose.yaw)
    radius = grid.resolution / 2
    a, b = policy.robot_length / 2, policy.robot_width / 2
    hx, hy = abs(co)*a + abs(si)*b, abs(si)*a + abs(co)*b
    samples = []
    for y, x in np.argwhere(cells == 1):
        dx = grid.origin[0] + (x+.5)*grid.resolution - pose.x
        dy = grid.origin[1] + (y+.5)*grid.resolution - pose.y
        forward, left = co*dx + si*dy, -si*dx + co*dy
        if (abs(dx) > hx+radius+1e-8 or abs(dy) > hy+radius+1e-8
                or abs(forward) > a+radius*(abs(co)+abs(si))+1e-8
                or abs(left) > b+radius*(abs(co)+abs(si))+1e-8):
            continue
        height = float(top[y, x]-floor_height[y, x])
        samples.append({'cell_xy': [int(x), int(y)], 'forward_m': float(forward),
                        'left_m': float(left),
                        'height_above_floor_m': height if math.isfinite(height) else None})
    if nominal['occupied_cells']:
        status = 'occupied_inside_nominal_body'
        reason = 'Занятость пересекает сам корпус: проверить привязку карты и источник отражений; уменьшение запаса проблему не решает'
    elif not nominal['clear']:
        status = 'nominal_body_not_observed'
        reason = 'Нет полной наблюдаемой опоры под корпусом'
    elif not expanded['clear']:
        status = 'clearance_margin_blocked'
        reason = 'Корпус свободен, но внешний запас пересекает препятствие или неизвестную область'
    else:
        status = 'body_clear'
        reason = 'Текущий корпус и запас помещаются; это не проверка всего маршрута'
    return {'status': status, 'reason': reason, 'nominal': nominal, 'expanded': expanded,
            'occupied_samples': samples, 'map_modified': False, 'motion_authorized': False}
