"""Регрессия торможения перед каждым мелким узлом сетки."""
import unittest
from wolf_go2.models import Grid,Observation,Pose,Policy
from wolf_go2.mission import MissionController

class NavigationDeadbandTests(unittest.TestCase):
    def test_dense_grid_does_not_keep_command_below_step_threshold(self):
        policy=Policy(max_vx=.30)
        controller=MissionController(policy);controller.phase='APPROACH';controller._dt=.1
        grid=Grid((-2.025,-2.025),.05,[[0]*100 for _ in range(100)],verified=True)
        x=0.
        for i in range(70):
            obs=Observation(1+i*.1,Pose(x,0,0,.31),grid,localized=True,localization_error_m=.02)
            decision=controller._navigate(obs,(1.5,0),'Тест плотной сетки')
            if decision is None:break
            # Модель мёртвой зоны проверяет алгоритм, не имитирует всю динамику Go2.
            if decision.vx>=.20:x+=decision.vx*.1
        self.assertGreater(x,.7)
