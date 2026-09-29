"""Некалиброванная проверка согласованности видео и полученной позы.

Неподвижное видео при изменении yaw может означать дрейф позы, зависание
видеопотока или несогласованные источники. Это диагностическое свидетельство,
а не установление аппаратной неисправности и не основание исправлять yaw.
Модуль не выполняет команды и не меняет переданную позу.
"""
from copy import deepcopy
import hashlib
import math
from numbers import Real
import time

import cv2
import numpy as np


def _finite(value):
    return isinstance(value, Real) and not isinstance(value, (bool, np.bool_)) and math.isfinite(value)


def _pose_values(pose):
    try:
        values = (pose.x, pose.y, pose.yaw)
        return (float(values[0]),float(values[1]),math.remainder(float(values[2]),2*math.pi)) if all(_finite(v) for v in values) else None
    except AttributeError:
        return None


def _pose_delta(a, b):
    yaw = math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
    return math.hypot(b[0]-a[0], b[1]-a[1]), abs(math.degrees(yaw))


class ConsistencyMonitor:
    """observe(image_bgr, pose, t) возвращает текущие диагностические коды.

    t — монотонное время получения согласованного пакета в секундах.
    Ожидается вызов примерно раз в 0.5 с; разрыв более 1.5 с сбрасывает
    непрерывность. Пиксельное свидетельство приводится к ширине 640.
    .report — отдельная JSON-совместимая копия измерений, не live-разрешение.
    """
    width = 640
    min_features = 40
    static_seconds = 5.0
    static_p95_px = 1.0
    yaw_threshold_deg = 2.0
    xy_threshold_m = .04
    duplicate_seconds = 3.0
    duplicate_motion_xy_m = .02
    duplicate_motion_yaw_deg = 1.0
    max_sample_gap = 1.5

    def __init__(self):
        # Ограничение разрешения, числа точек и уровней ограничивает стоимость.
        # Реальный бюджет на Pi измеряется отдельно; здесь он не гарантируется.
        self._orb = cv2.ORB_create(nfeatures=400, nlevels=4, fastThreshold=16)
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self._anchor = None
        self._last_t = None
        self._last_pose = None
        self._last_digest = None
        self._duplicate_since = None
        self._duplicate_pose = None
        self._duplicate_motion_xy = 0.
        self._duplicate_motion_yaw = 0.
        self._report = self._empty_report()

    def _empty_report(self):
        return {
            'diagnostic_only': True,
            'hardware_verdict': None,
            'correction_applied': False,
            'evidence_scope': 'pixel_correspondence_and_received_pose_only',
            'status': 'awaiting_image',
            'reasons': [],
            'normalized_width_px': self.width,
            'tracked_features': 0,
            'p95_displacement_px': None,
            'static_duration_s': 0.,
            'static_samples': 0,
            'pose_yaw_change_deg': 0.,
            'pose_xy_change_m': 0.,
            'exact_duplicate_duration_s': 0.,
            'pose_change_with_exact_same_frame': False,
            'last_pose_delta_xy_m': None,
            'last_pose_delta_yaw_deg': None,
            'processing_ms': 0.,
            'thresholds': {
                'minimum_tracked_features': self.min_features,
                'static_duration_s': self.static_seconds,
                'p95_displacement_px_strictly_below': self.static_p95_px,
                'yaw_change_deg_strictly_above': self.yaw_threshold_deg,
                'xy_change_m_strictly_below': self.xy_threshold_m,
                'duplicate_duration_s_strictly_above': self.duplicate_seconds,
                'duplicate_motion_xy_m_strictly_above': self.duplicate_motion_xy_m,
                'duplicate_motion_yaw_deg_strictly_above': self.duplicate_motion_yaw_deg,
                'max_sample_gap_s': self.max_sample_gap,
            },
            'limitations': 'Без калибровки, синхронизации и независимой проверки источников причина расхождения не устанавливается.',
        }

    @property
    def report(self):
        return deepcopy(self._report)

    def _reset_window(self):
        self._anchor = None
        self._last_t = None
        self._last_pose = None
        self._last_digest = None
        self._duplicate_since = None
        self._duplicate_pose = None
        self._duplicate_motion_xy = self._duplicate_motion_yaw = 0.

    def _features(self, gray):
        points, descriptors = self._orb.detectAndCompute(gray, None)
        coordinates = np.asarray([point.pt for point in points], dtype=np.float32).reshape(-1,2)
        return coordinates, descriptors

    def _set_anchor(self, coordinates, descriptors, pose, t):
        self._anchor = {
            'points': coordinates, 'descriptors': descriptors, 'pose': pose, 't': t,
            'ids': set(range(len(coordinates))), 'samples': 1, 'max_xy': 0.,
        }

    def observe(self, image_bgr, pose, t) -> list[str]:
        started = time.perf_counter()
        report = self._empty_report()
        received_pose = _pose_values(pose)
        if (not _finite(t) or received_pose is None or not isinstance(image_bgr,np.ndarray)
                or image_bgr.dtype != np.uint8 or image_bgr.ndim != 3
                or image_bgr.shape[2] != 3 or min(image_bgr.shape[:2]) < 16):
            self._reset_window()
            report['status'] = 'invalid_input'
            report['processing_ms'] = (time.perf_counter()-started)*1000
            self._report = report
            return []
        t = float(t)
        previous_poses = [self._last_pose,self._duplicate_pose,
                          self._anchor['pose'] if self._anchor else None]
        if any(previous is not None and not math.isfinite(_pose_delta(previous,received_pose)[0])
               for previous in previous_poses):
            self._reset_window()
            report['status'] = 'invalid_input'
            report['processing_ms'] = (time.perf_counter()-started)*1000
            self._report = report
            return []
        discontinuity = self._last_t is not None and (t <= self._last_t or t-self._last_t > self.max_sample_gap)
        if discontinuity:
            self._reset_window()
        # Хеш исходного BGR отличает побайтный повтор от неподвижной сцены.
        source = np.ascontiguousarray(image_bgr)
        digest = (source.shape, hashlib.sha256(memoryview(source).cast('B')).digest())
        identical = digest == self._last_digest
        if identical:
            xy,yaw = _pose_delta(self._duplicate_pose,received_pose)
            self._duplicate_motion_xy = max(self._duplicate_motion_xy,xy)
            self._duplicate_motion_yaw = max(self._duplicate_motion_yaw,yaw)
            report['exact_duplicate_duration_s'] = t-self._duplicate_since
        else:
            self._duplicate_since, self._duplicate_pose = t,received_pose
            self._duplicate_motion_xy = self._duplicate_motion_yaw = 0.
        if self._last_pose is not None:
            xy,yaw = _pose_delta(self._last_pose,received_pose)
            report['last_pose_delta_xy_m'] = xy
            report['last_pose_delta_yaw_deg'] = yaw
            report['pose_change_with_exact_same_frame'] = bool(identical and (xy > 1e-8 or yaw > 1e-6))
        duplicate_motion = (self._duplicate_motion_xy > self.duplicate_motion_xy_m
                            or self._duplicate_motion_yaw > self.duplicate_motion_yaw_deg)
        if report['exact_duplicate_duration_s'] > self.duplicate_seconds and duplicate_motion:
            report['reasons'].append('duplicate_video_during_pose_motion')

        # Портретный вход тоже не увеличивает рабочее изображение без границ.
        # Координаты признаков затем выражаются в единицах ширины 640.
        scale = min(self.width/source.shape[1],480/source.shape[0])
        work_width = max(16,round(source.shape[1]*scale))
        height = max(16,round(source.shape[0]*scale))
        normalized = cv2.resize(source,(work_width,height),interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(normalized,cv2.COLOR_BGR2GRAY)
        coordinates,descriptors = self._features(gray)
        coordinates = coordinates * (self.width/work_width)
        report['status'] = 'warming_up'
        if self._anchor is None:
            self._set_anchor(coordinates,descriptors,received_pose,t)
        else:
            anchor = self._anchor
            matches = []
            if descriptors is not None and anchor['descriptors'] is not None:
                matches = [m for m in self._matcher.match(anchor['descriptors'],descriptors)
                           if m.distance <= 40 and m.queryIdx in anchor['ids']]
            displacement = np.asarray([np.linalg.norm(coordinates[m.trainIdx]-anchor['points'][m.queryIdx])
                                       for m in matches],dtype=float)
            count = len(matches)
            p95 = float(np.percentile(displacement,95)) if count else None
            xy,yaw = _pose_delta(anchor['pose'],received_pose)
            max_xy = max(anchor['max_xy'],xy)
            report.update(tracked_features=count,p95_displacement_px=p95,
                          pose_yaw_change_deg=yaw,pose_xy_change_m=xy)
            if count >= self.min_features and p95 < self.static_p95_px and max_xy < self.xy_threshold_m:
                anchor['ids'].intersection_update(m.queryIdx for m in matches)
                anchor['samples'] += 1
                anchor['max_xy'] = max_xy
                report['static_duration_s'] = t-anchor['t']
                report['static_samples'] = anchor['samples']
                report['status'] = 'static_pixel_evidence'
                if (report['static_duration_s'] >= self.static_seconds
                        and yaw > self.yaw_threshold_deg+1e-9):
                    report['reasons'].append('orientation_or_video_inconsistency')
            else:
                # Любое движение изображения, недостаток точек или выход XY
                # прерывает окно. Возврат изображения/позы не склеивает окна.
                report['status'] = 'insufficient_features' if count < self.min_features else 'scene_or_position_changed'
                self._set_anchor(coordinates,descriptors,received_pose,t)
        if discontinuity:
            report['status'] = 'time_discontinuity_reset'
        self._last_t,self._last_pose,self._last_digest = t,received_pose,digest
        report['processing_ms'] = (time.perf_counter()-started)*1000
        self._report = report
        return list(report['reasons'])
