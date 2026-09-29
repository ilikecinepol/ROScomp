"""Профиль назначенного робота содержит измерения и пределы, но не карту арены."""
from dataclasses import dataclass, field, fields
from pathlib import Path
import json
import math
from .models import Policy

@dataclass
class RobotProfile:
    robot_id: str
    policy: Policy = field(default_factory=Policy)
    schema_version: int = 1
    verified: dict = field(default_factory=dict)
    geometry: dict = field(default_factory=dict)
    time_contracts: dict = field(default_factory=dict)
    expected_identity: dict = field(default_factory=dict)
    evidence: dict = field(default_factory=dict)

    @classmethod
    def load(cls, path):
        obj = json.loads(Path(path).read_text(encoding='utf-8-sig'))
        if not isinstance(obj, dict) or set(obj)-{f.name for f in fields(cls)}:
            raise ValueError('Неизвестные поля профиля; координат арены здесь быть не должно')
        policy = obj.pop('policy', {})
        if not isinstance(policy, dict) or set(policy)-{f.name for f in fields(Policy)}:
            raise ValueError('Неизвестные параметры Policy')
        if 'validated_skills' in policy:
            policy['validated_skills'] = tuple(policy['validated_skills'])
        result = cls(**obj, policy=Policy(**policy))
        result.validate()
        return result

    def validate(self):
        self.policy.validate()
        if self.schema_version != 1 or not isinstance(self.robot_id, str) or not self.robot_id.strip():
            raise ValueError('Нет идентификатора назначенного робота или версии профиля')
        for name in ('verified', 'geometry', 'time_contracts', 'expected_identity', 'evidence'):
            if not isinstance(getattr(self, name), dict):
                raise ValueError(name + ' должен быть объектом')
        forbidden = {'obstacles', 'waypoints', 'route', 'arena', 'structure_evidence'}
        if set(self.geometry) & forbidden:
            raise ValueError('Профиль не должен содержать разметку препятствий')
        for name, contract in self.time_contracts.items():
            if not isinstance(contract, dict):
                raise ValueError('Некорректный временной контракт ' + name)
            for key in ('scale', 'offset'):
                value = contract.get(key)
                if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                    raise ValueError('Нет проверенного преобразования времени ' + name)
            if contract['scale'] <= 0 or contract.get('verified') is not True or not contract.get('evidence'):
                raise ValueError('Временной контракт не подтверждён: ' + name)
            if contract.get('clock_domain') != 'linux_monotonic' or not contract.get('pi_boot_id'):
                raise ValueError('Контракт времени должен относиться к конкретной загрузке Pi: '+name)
        return self

    def readiness(self, assigned_robot_id):
        reasons = []
        if self.robot_id != assigned_robot_id:
            reasons.append('Профиль относится к другому назначенному роботу')
        for key in ('identity', 'motion_response', 'geometry_frames', 'localization', 'support_geometry'):
            if self.verified.get(key) is not True or not self.evidence.get(key):
                reasons.append('Нет подтверждения: ' + key)
        for key in ('ROBOTODOM', 'ULIDAR_ARRAY'):
            if key not in self.time_contracts:
                reasons.append('Нет проверенной шкалы времени: ' + key)
        if not self.expected_identity.get('serial_number'):
            reasons.append('Нет уникального серийного номера; версия прошивки не различает собак')
        if self.geometry.get('pose_child_frame') != 'base_link':
            reasons.append('Не подтверждена поза base_link; неизвестный child_frame не угадывается')
        if self.geometry.get('point_frame') != self.geometry.get('pose_frame'):
            reasons.append('Карта и поза не в одной подтверждённой системе')
        if self.geometry.get('up_axis') != '+z' or self.geometry.get('units') != 'm':
            reasons.append('Не подтверждены единицы и вертикальная ось')
        return reasons

    def session_errors(self, assigned_robot_id, identity, boot_id):
        reasons = self.readiness(assigned_robot_id)
        for key, expected in self.expected_identity.items():
            if str(identity.get(key, '')) != str(expected):
                reasons.append('Не совпадает идентификация робота: '+key)
        for name, contract in self.time_contracts.items():
            if contract.get('pi_boot_id') != boot_id:
                reasons.append('Временная калибровка относится к другой загрузке Pi: '+name)
        return reasons

    def geometry_contract(self, assigned_robot_id, acquisition_age):
        errors = self.readiness(assigned_robot_id)
        return {**self.geometry, 'geometry_frames_validated': not errors,
                'identity_verified': not errors, 'source_time_validated': not errors,
                'acquisition_age_s': acquisition_age}
