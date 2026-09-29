"""Выделение переднего ската из точек, независимо от связности матов.

Результат — диагностическая гипотеза плоскости, не маршрут и не разрешение
движения. Система координат: x вперёд, y влево, z вверх относительно пола.
"""
import math
import numpy as np


def detect_ramp(points, seed=7):
    cloud = np.asarray(points, dtype=float)
    if cloud.ndim != 2 or cloud.shape[1] != 3:
        raise ValueError('Нужен массив Nx3')
    valid = np.isfinite(cloud).all(axis=1)
    valid &= (cloud[:, 0] > .05) & (cloud[:, 0] < 3.)
    valid &= (abs(cloud[:, 1]) < 1.) & (cloud[:, 2] > .10) & (cloud[:, 2] < 1.)
    cloud = cloud[valid]
    if len(cloud) < 40:
        return None
    rng = np.random.default_rng(seed)
    if len(cloud) > 12000:
        cloud = cloud[rng.choice(len(cloud), 12000, replace=False)]
    matrix = np.column_stack((cloud[:, :2], np.ones(len(cloud))))
    best = None
    for _ in range(220):
        indices = rng.choice(len(cloud), 3, replace=False)
        sample = matrix[indices]
        if abs(np.linalg.det(sample)) < .01:
            continue
        coef = np.linalg.solve(sample, cloud[indices, 2])
        if not math.tan(math.radians(10)) <= coef[0] <= math.tan(math.radians(40)) or abs(coef[1]) > .3:
            continue
        mask = abs(matrix@coef-cloud[:, 2]) < .04
        if mask.sum() < 40:
            continue
        support = cloud[mask]
        spans = np.quantile(support[:, :2], .95, axis=0)-np.quantile(support[:, :2], .05, axis=0)
        if spans[0] < .25 or spans[1] < .25:
            continue
        # Передняя рампа имеет приоритет над дальней наклонной комбинацией
        # точек других объектов. Это выбор гипотезы, не оценка проходимости.
        score = float(mask.sum())/(1.+float(np.median(support[:,0])))**2
        if best is None or score > best[0]:
            best = (score, mask)
    if best is None:
        return None
    support = cloud[best[1]]
    coef = np.linalg.lstsq(matrix[best[1]], support[:, 2], rcond=None)[0]
    if not math.tan(math.radians(10)) <= coef[0] <= math.tan(math.radians(40)) or abs(coef[1]) > .3:
        return None
    low, high = np.quantile(support[:, :2], [.05, .95], axis=0)
    return dict(kind='ascending_plane_hypothesis', metric_target=False,
                motion_authorized=False, coefficients_z_ax_by_c=coef.tolist(),
                longitudinal_slope_deg=math.degrees(math.atan(coef[0])),
                observed_bounds_xy=[low.tolist(), high.tolist()],
                inliers=len(support), residual_p95_m=float(np.quantile(abs(matrix[best[1]]@coef-support[:, 2]), .95)),
                limitations=['Границы наблюдений не равны физическим краям настила',
                             'Плоскость не подтверждает ступени, выход или свежесть карты'])
