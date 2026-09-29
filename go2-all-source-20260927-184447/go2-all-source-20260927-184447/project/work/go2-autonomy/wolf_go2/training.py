"""Одно нажатие — один навык, затем необязательный подход без прохождения."""
from dataclasses import replace
from .mission import MissionController

KINDS = ('slalom', 'aframe', 'teeter', 'platforms')


class TrainingController:
    def __init__(self, policy, kind, stage_next=None, commissioning=False):
        if kind not in KINDS or stage_next not in (None, *KINDS) or kind == stage_next:
            raise ValueError('Некорректное испытание или следующий объект')
        self.policy, self.kind, self.stage_next = policy, kind, stage_next
        self.commissioning=bool(commissioning)
        self.controller = MissionController(policy, trial_kind=kind, commissioning=self.commissioning)
        self.staging = False
        self.completion_order = []
        self.epoch = None

    @property
    def phase(self):
        return self.controller.phase

    def step(self, obs, start=False):
        if self.epoch is not None and obs.frame_epoch != self.epoch:
            return self.controller._fault('Сменилась система координат между этапами испытания')
        if start and self.epoch is None:
            self.epoch = obs.frame_epoch
        decision = self.controller.step(obs, start=start)
        if not self.staging:
            self.completion_order = list(self.controller.completion_order)
        if decision.phase == 'FINISHED' and self.stage_next and not self.staging:
            self.staging = True
            self.controller = MissionController(self.policy, trial_kind=self.stage_next, approach_only=True, commissioning=self.commissioning)
            # На границе всегда нулевая команда. Следующий объект не засчитывается.
            return replace(decision, phase='WAIT_START', reason='Препятствие пройдено; подготовка подхода к '+self.stage_next)
        if self.staging:
            return replace(decision, completed=tuple(self.completion_order), reason='Подход к следующему: '+decision.reason)
        return decision
