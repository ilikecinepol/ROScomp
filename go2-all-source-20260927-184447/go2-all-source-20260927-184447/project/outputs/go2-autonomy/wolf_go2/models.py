"""Общий контракт планировщика, датчиков и проверки команд. Единицы СИ."""
from dataclasses import dataclass, field
from typing import Optional
import math

KINDS = ('aframe', 'bridge', 'teeter', 'slalom', 'platforms')

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    z: float = 0.0

@dataclass
class Grid:
    """cells[y][x]: -1 неизвестно, 0 подтверждённый проход, 1 препятствие.

    Незаполненная ячейка карты занятости НЕ становится проходом.
    resolution в метрах; origin — нижний левый угол ячейки [0][0].
    """
    origin: tuple[float, float]
    resolution: float
    cells: list[list[int]]
    frame_epoch: int = 0
    verified: bool = False

@dataclass(frozen=True)
class Obstacle:
    """Маршрут по поверхности строится из наблюдений, а не файла раскладки.

    path = (x,y,z) от входа к выходу. Подтверждение геометрии означает
    проверку наблюдений; физические пределы задаёт отдельный профиль.
    """
    id: str
    kind: str
    path: tuple[tuple[float, float, float], ...]
    width: float
    observed_at: float
    confidence: float = 0.0
    confirmations: int = 0
    geometry_verified: bool = False
    direction_verified: bool = False
    bidirectional: bool = False
    max_slope_deg: float = 0.0
    max_step_m: float = 0.0
    max_gap_m: float = 0.0
    max_lip_m: float = 0.0
    frame_epoch: int = 0
    evidence: tuple[str, ...] = ()

@dataclass(frozen=True)
class StartLine:
    """Наблюдаемый конечный отрезок стартовой линии в системе карты.

    endpoints в метрах, observed_at в шкале Observation.t. Это контракт
    восприятия, а не заранее известная координата или имитация датчика.
    Достоверность, свежесть и совпадение эпох проверяет потребитель.
    """
    endpoints: tuple[tuple[float, float], tuple[float, float]]
    observed_at: float
    confidence: float = 0.0
    frame_epoch: int = 0
    geometry_verified: bool = False
    confirmations: int = 0

@dataclass
class Observation:
    t: float
    pose: Optional[Pose]
    grid: Optional[Grid]
    obstacles: list[Obstacle] = field(default_factory=list)
    localized: bool = False
    frame_epoch: int = 0
    localization_error_m: float = math.inf
    start_line_visible: bool = False
    start_line_crossed: bool = False
    roll: float = 0.0
    pitch: float = 0.0
    body_height: float = 0.0
    speed: float = 0.0
    support_height_m: Optional[float] = None
    support_verified: bool = False
    diagnostics: list[str] = field(default_factory=list)
    start_line: Optional[StartLine] = None
    # Отдельная наблюдённая опора выбранной поверхности. Остальные поднятые
    # объекты и неизвестные ячейки сохраняют запрет обычной карты пола.
    traversal_grids: dict[str, Grid] = field(default_factory=dict)

@dataclass(frozen=True)
class Decision:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    phase: str = 'WAIT_START'
    reason: str = ''
    target_id: Optional[str] = None
    completed: tuple[str, ...] = ()

    @property
    def moving(self):
        return any(abs(v) > 1e-9 for v in (self.vx, self.vy, self.wz))

@dataclass
class Policy:
    """Значения — консервативные стартовые настройки, не паспортные гарантии."""
    max_vx: float = 0.35
    max_vy: float = 0.15
    max_wz: float = 0.5
    traverse_speed: float = 0.12
    max_accel: float = 0.25
    max_yaw_accel: float = 0.6
    robot_width: float = 0.40
    robot_length: float = 0.75
    clearance: float = 0.10
    waypoint_tolerance: float = 0.10
    alignment_tolerance: float = 0.08
    max_localization_error: float = 0.08
    max_observation_age: float = 0.6
    max_candidate_age: float = 2.0
    max_step: float = 0.12
    max_slope_deg: float = 28.0
    max_gap: float = 0.02
    max_lip: float = 0.02
    max_roll_deg: float = 15.0
    max_pitch_deg: float = 35.0
    minimum_body_height: float = 0.22
    max_body_height: float = 0.48
    min_battery: float = 25.0
    max_motor_temperature: float = 60.0
    max_mission_seconds: float = 600.0
    verify_seconds: float = 1.0
    progress_timeout: float = 12.0
    # Только физически испытанные на назначенном роботе навыки включаются в live.
    validated_skills: tuple[str, ...] = ()

    def validate(self):
        for key, value in vars(self).items():
            if key == 'validated_skills':
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('Недопустимый параметр Policy: ' + key)
        if self.max_vx > .6 or self.max_vy > .5 or self.max_wz > 1.0:
            raise ValueError('Превышены пределы предоставленного API 0.6/0.5/1.0')
        if self.traverse_speed > self.max_vx or self.minimum_body_height >= self.max_body_height:
            raise ValueError('Противоречивые пределы скорости/высоты')
        if any(kind not in KINDS for kind in self.validated_skills):
            raise ValueError('Неизвестный вид препятствия')
        return self
