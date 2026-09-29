"""Ряд цилиндрических стоек в основном восприятии, без дорисовки скрытых стоек."""
import numpy as np
from .poles import slalom_pole_candidates, pole_rows, exclude_body_conflicts
from .slalom_route import plan_slalom


def cylinder_rows(cloud, floor, resolution, grid, pose, policy, uncertainty):
    poles = slalom_pole_candidates(cloud, floor, resolution)
    poles = exclude_body_conflicts(poles, pose, policy, uncertainty)
    result = []
    for row in pole_rows(poles):
        selected = [poles[i] for i in row['pole_indices']]
        centers = np.asarray(row['centers_xy'])
        radius = max(max(p['width_xy_m'])/2 for p in selected)
        plan = plan_slalom(grid, centers, radius, pose, policy, uncertainty)
        center = centers.mean(0)
        path = [[float(x), float(y), float(np.dot([x,y,1.],floor))] for x,y in plan['path']]
        reasons = ['Принадлежность стоек змейке и полнота ряда ещё не подтверждены камерой']
        if not path:
            reasons.append('Нет свободного маршрута вокруг наблюдённых стоек')
        result.append({
            'kind':'slalom', 'candidate_type':'cylindrical_pole_row',
            'center':[*center.tolist(),float(np.dot([*center,1.],floor))],
            'width_m':float(2*(radius+policy.robot_width/2+policy.clearance+uncertainty)),
            'strong_geometry':False, 'path':path, 'max_slope_deg':0.,
            'max_step_m':0., 'max_gap_m':0., 'max_lip_m':0.,
            'bidirectional':False, 'reasons':reasons,
            'evidence':['Наблюдён ряд отдельных вертикальных цилиндрических компонентов'],
            'observed_poles':len(selected), 'pole_centers_xy':centers.tolist(),
            'row_complete_verified':False, 'route_preview':plan,
        })
    return poles, result
