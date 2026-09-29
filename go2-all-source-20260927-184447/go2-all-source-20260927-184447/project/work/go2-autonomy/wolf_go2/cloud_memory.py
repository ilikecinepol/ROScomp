"""Короткая память измерений в одной проверенной системе координат."""
from collections import deque
import math
import numpy as np


class CloudMemory:
    def __init__(self):
        self.frames=deque(maxlen=4)
        self.signature=None

    def clear(self):
        self.frames.clear()
        self.signature=None

    def merge(self,points,acquired_at,now,window,max_age,signature):
        if not all(math.isfinite(v) for v in (acquired_at,now,window,max_age)) or not 0<window<=max_age:
            self.clear()
            raise ValueError('Недопустимое время памяти облака')
        if not 0<=now-acquired_at<=max_age:
            self.clear()
            raise ValueError('Текущее облако устарело')
        cloud=np.asarray(points,dtype=float)
        if cloud.ndim!=2 or cloud.shape[1]!=3 or not np.isfinite(cloud).all():
            self.clear()
            raise ValueError('Некорректные точки памяти облака')
        if signature!=self.signature:
            self.clear();self.signature=signature
        if self.frames and acquired_at<self.frames[-1][0]:
            self.clear()
            raise ValueError('Время облака пошло назад')
        while self.frames and now-self.frames[0][0]>min(window,max_age):
            self.frames.popleft()
        # Повтор того же измерения не продлевает срок хранения.
        if not self.frames or acquired_at>self.frames[-1][0]:
            self.frames.append((acquired_at,cloud.copy()))
        # Объединение не удаляет препятствия и не заполняет невиданные области.
        # Точки, исчезнувшие из нового кадра, живут не дольше заданного окна.
        merged=np.unique(np.concatenate([p for _,p in self.frames]+[cloud]),axis=0)
        return merged
