"""Устойчивость пар камера–лидар; сама по себе не подтверждает калибровку."""
import math
import numpy as np
from .association import gated_assignment


class PairPersistence:
    def __init__(self, radius=.15, max_gap=2.):
        if not math.isfinite(radius) or radius<=0 or not math.isfinite(max_gap) or max_gap<=0:
            raise ValueError('Нужны положительные пороги')
        self.radius=radius
        self.max_gap=max_gap
        self.context=None
        self.last_time=None
        self.last_camera=None
        self.tracks=[]
        self.next_id=0

    def update(self, context, received, camera_received, points):
        xy=np.asarray(points,dtype=float).reshape((-1,2))
        if not np.isfinite(xy).all() or not math.isfinite(received) or not math.isfinite(camera_received):
            raise ValueError('Некорректные координаты или время')
        if context!=self.context or (self.last_time is not None and received<self.last_time):
            self.tracks=[]
            self.last_time=self.last_camera=None
            self.context=context
        # Один и тот же кадр или пакет не добавляет подтверждений.
        if self.last_time is not None and (received==self.last_time or camera_received<=self.last_camera):
            return []
        if self.last_time is not None and received-self.last_time>self.max_gap:
            self.tracks=[]
        costs=np.array([[np.linalg.norm(point-np.asarray(t['anchor_xy'])) for t in self.tracks] for point in xy])
        costs=costs.reshape((len(xy),len(self.tracks)))
        pairs=gated_assignment(costs,maximum=self.radius,ambiguity_margin=self.radius*.1)
        by_point={p['image_index']:self.tracks[p['projected_index']] for p in pairs if p['accepted']}
        current=[]
        for i,point in enumerate(xy):
            previous=by_point.get(i)
            if previous is None:
                identity=self.next_id
                self.next_id+=1
                count=1
            else:
                identity=previous['id']
                count=previous['consecutive_frames']+1
            current.append({'id':identity,'xy':point.tolist(),'consecutive_frames':count,
                            'anchor_xy':previous['anchor_xy'] if previous else point.tolist(),
                            'calibration_verified':False})
        self.tracks=current
        self.last_time=received
        self.last_camera=camera_received
        return [dict(t) for t in current]
