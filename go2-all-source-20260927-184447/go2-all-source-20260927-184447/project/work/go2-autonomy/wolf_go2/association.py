"""Сопоставление объектов с отказом от далёких и неоднозначных пар."""
import numpy as np
from scipy.optimize import linear_sum_assignment


def gated_assignment(costs, maximum=35., ambiguity_margin=5.):
    costs=np.asarray(costs,dtype=float)
    if costs.ndim!=2 or not np.isfinite(costs).all() or np.any(costs<0):
        raise ValueError('Нужна конечная неотрицательная матрица расстояний')
    if not np.isfinite([maximum,ambiguity_margin]).all() or maximum<=0 or ambiguity_margin<0:
        raise ValueError('Некорректные пределы сопоставления')
    n,m=costs.shape
    if not n or not m:
        return []
    # Индивидуальные фиктивные столбцы позволяют оставить силуэт без пары.
    augmented=np.full((n,m+n),maximum+1e-6)
    augmented[:,:m]=np.where(costs<=maximum,costs,maximum*1000+1)
    rows,cols=linear_sum_assignment(augmented)
    best=float(augmented[rows,cols].sum())
    result=[]
    for i,j in zip(rows,cols):
        if j>=m or costs[i,j]>maximum:
            continue
        # Запрет выбранной пары и повторное оптимальное назначение проверяет
        # конкуренцию всего набора, включая возможность оставить объект без пары.
        alternative=augmented.copy()
        alternative[i,j]=maximum*1000+1
        ar,ac=linear_sum_assignment(alternative)
        margin=float(alternative[ar,ac].sum()-best)
        result.append({'image_index':int(i),'projected_index':int(j),
                       'pixel_residual':float(costs[i,j]),'alternative_margin_px':margin,
                       'accepted':bool(margin>=ambiguity_margin)})
    return result
