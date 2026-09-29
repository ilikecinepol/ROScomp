"""Ограниченная синтетика FSM; не является испытанием физического Go2."""
import math
import random
import unittest
from dataclasses import replace

from wolf_go2.models import Grid, KINDS, Obstacle, Observation, Policy, Pose, StartLine
from wolf_go2.mission import MissionController


def surface(kind='bridge', name=None, origin=(1.2, 0.), yaw=0.):
    profiles = {
        'bridge': ((0., 0., 0.), (.4, 0., 0.), (.8, 0., 0.)),
        'teeter': ((0., 0., 0.), (.4, 0., .08), (.8, 0., 0.)),
        'aframe': ((0., 0., 0.), (.4, 0., .12), (.8, 0., 0.)),
        'slalom': ((0., 0., 0.), (.35, .2, 0.), (.7, -.2, 0.), (1.05, .2, 0.), (1.4, 0., 0.)),
        'platforms': ((0., 0., 0.), (.3, 0., .10), (.6, 0., 0.), (.9, 0., .10), (1.2, 0., 0.)),
    }
    path = tuple((origin[0]+x*math.cos(yaw)-y*math.sin(yaw),
                  origin[1]+x*math.sin(yaw)+y*math.cos(yaw), z)
                 for x, y, z in profiles[kind])
    return Obstacle(name or kind+'-seen', kind, path, 1.2, 0., .95, 3,
                    True, True, False, max_step_m=.1 if kind == 'platforms' else 0.)


class SensorFixture:
    """Отдельная синтетическая последовательность физических поз и опоры.

    Она проверяет переходы автомата, а не динамику робота или качество датчика.
    Маршрут и новые наблюдения задаются явно; поза никогда не телепортируется.
    """
    def __init__(self, items=(), policy=None):
        self.policy = policy or Policy(validated_skills=KINDS, required_kinds=KINDS, return_mode='line', max_mission_seconds=1200.)
        self.controller = MissionController(self.policy)
        self.grid = Grid((-4., -4.), .25, [[0]*48 for _ in range(48)], verified=True)
        self.items = list(items)
        self.t = 0.
        self.pose = Pose(.125, .125, 0., z=7.4)
        self.support = 0.
        self.last = None
        self._observed_pose = None
        self.line_endpoints = ((.375,-1.5),(.375,1.5))
        self.traversal_grids = {}

    def tick(self, start=False, fresh=True, **changes):
        self.t += .2
        items = [replace(v, observed_at=self.t) for v in self.items] if fresh else self.items
        crossed = False
        if self._observed_pose is not None:
            a,b = self._observed_pose.x-.375,self.pose.x-.375
            if a < 0 <= b or b < 0 <= a:
                fraction = a/(a-b)
                y = self._observed_pose.y+fraction*(self.pose.y-self._observed_pose.y)
                crossed = -1.5 <= y <= 1.5
        obs = Observation(self.t, self.pose, self.grid, items, localized=True,
            localization_error_m=.02, start_line_visible=True,
            start_line_crossed=crossed,body_height=.32, support_height_m=self.support, support_verified=True,
            start_line=StartLine(self.line_endpoints,self.t,confidence=1.,geometry_verified=True,confirmations=4),
            traversal_grids=self.traversal_grids)
        for key, value in changes.items():
            setattr(obs, key, value)
        self.last = self.controller.step(obs, start=start)
        self._observed_pose = self.pose
        assert math.isfinite(self.last.vx) and math.isfinite(self.last.wz)
        assert 0 <= self.last.vx <= self.policy.max_vx
        assert abs(self.last.wz) <= self.policy.max_wz
        assert self.last.vy == 0.
        return self.last

    def advance_to(self, point, yaw=None):
        dx, dy = point[0]-self.pose.x, point[1]-self.pose.y
        distance = math.hypot(dx, dy)
        fraction = min(1., .06/max(distance, 1e-9))
        heading = self.pose.yaw if yaw is None else yaw
        error = math.atan2(math.sin(heading-self.pose.yaw), math.cos(heading-self.pose.yaw))
        heading = self.pose.yaw + min(.08, max(-.08, error))
        self.pose = Pose(self.pose.x+dx*fraction, self.pose.y+dy*fraction, heading, self.pose.z)
        if len(point) == 3:
            self.support += min(.025, max(-.025, point[2]-self.support))

    def until_traverse(self):
        if self.controller.phase == 'WAIT_START':
            self.tick(start=True)
        for _ in range(500):
            phase = self.controller.phase
            if phase == 'TRAVERSE':
                return
            if phase == 'APPROACH':
                c = self.controller
                target = c._nav_path[c._nav_index] if c._nav_path else c._approach_goal
                self.advance_to(target)
            elif phase == 'ALIGN':
                a, b = self.controller.current.path[:2]
                self.advance_to((self.pose.x,self.pose.y), math.atan2(b[1]-a[1],b[0]-a[0]))
            self.tick()
            if self.controller.phase == 'FAULT':
                raise AssertionError(self.last.reason)
        raise AssertionError('Автомат не дошёл до TRAVERSE: '+str(self.last))


class MissionTests(unittest.TestCase):
    def raised_fixture(self):
        item = replace(surface('aframe'),width=1.5)
        f = SensorFixture([item])
        for y in range(13,19):
            for x in range(21,24):
                f.grid.cells[y][x] = 1
        own = Grid(f.grid.origin,f.grid.resolution,[row[:] for row in f.grid.cells],verified=True)
        for y in range(13,19):
            for x in range(21,24):
                own.cells[y][x] = 0
        f.traversal_grids[item.id] = own
        return f

    def test_explicit_start_and_visible_line_required(self):
        f = SensorFixture([surface()])
        self.assertEqual(f.tick().phase, 'WAIT_START')
        self.assertEqual(f.tick(start=True,start_line_visible=False).phase, 'WAIT_START')
        self.assertEqual(f.tick(start=True).phase, 'DISCOVER')
        self.assertEqual(f.controller.start_pose, f.pose)

    def test_invalid_or_unknown_map_never_starts_blind_motion(self):
        for value in (-1, 1):
            f = SensorFixture([surface()])
            f.grid.cells = [[value]*48 for _ in range(48)]
            result = f.tick(start=True)
            self.assertFalse(result.moving)
            self.assertEqual(result.phase, 'WAIT_START')
        f = SensorFixture([surface()])
        f.grid.verified = False
        self.assertFalse(f.tick(start=True).moving)

    def test_default_profile_does_not_enable_obstacle_skill(self):
        f = SensorFixture([surface()], Policy())
        f.tick(start=True)
        self.assertEqual(f.tick().phase, 'DISCOVER')
        self.assertIsNone(f.controller.current)

    def test_candidate_gates_fail_closed(self):
        changes = [dict(confidence=float('nan')), dict(confidence=.79), dict(confirmations=2),
            dict(confirmations='3'), dict(geometry_verified=False), dict(direction_verified=False),
            dict(width=.63), dict(max_step_m=.121), dict(max_gap_m=.021),
            dict(max_lip_m=.021), dict(max_slope_deg=29.), dict(path=((1.,0.,0.),)),
            dict(path=None), dict(frame_epoch=9)]
        for change in changes:
            with self.subTest(change=change):
                f = SensorFixture([replace(surface(), **change)])
                f.tick(start=True)
                self.assertFalse(f.tick().moving)
                self.assertIsNone(f.controller.current)

    def test_obstacle_path_cannot_use_unknown_cells(self):
        f = SensorFixture([surface()])
        x, y, _ = f.items[0].path[1]
        f.grid.cells[int((y+4)/.25)][int((x+4)/.25)] = -1
        f.tick(start=True)
        f.tick()
        self.assertIsNone(f.controller.current)

    def test_stale_target_stops_immediately(self):
        f = SensorFixture([surface()])
        f.until_traverse()
        f.items = [replace(f.items[0], observed_at=f.t-3.)]
        # Старый пакет не заменяет более свежую запись памяти. Дожидаемся
        # истечения последнего действительно полученного наблюдения.
        for _ in range(12):
            decision = f.tick(fresh=False)
        self.assertFalse(decision.moving)
        self.assertIn('Устаревшее', decision.reason)
        self.assertEqual(f.controller.completed_ids, set())

    def test_frame_epoch_fault_latched_even_with_new_start(self):
        f = SensorFixture([surface()])
        f.tick(start=True)
        self.assertEqual(f.tick(frame_epoch=1).phase, 'FAULT')
        self.assertFalse(f.tick(start=True).moving)
        self.assertEqual(f.controller.phase, 'FAULT')

    def test_position_and_yaw_jump_faults(self):
        for pose in (Pose(3.,0.,0.), Pose(.125,.125,1.5)):
            with self.subTest(pose=pose):
                f = SensorFixture([surface()])
                f.tick(start=True)
                f.pose = pose
                self.assertEqual(f.tick().phase, 'FAULT')
                self.assertEqual(f.tick(start=True).phase, 'FAULT')

    def test_missing_support_cannot_be_replaced_with_pose_z_or_body_height(self):
        f = SensorFixture([surface('platforms')])
        f.until_traverse()
        self.assertFalse(f.tick(support_height_m=None,support_verified=False).moving)
        self.assertEqual(f.controller.waypoint_index, 0)
        self.assertEqual(f.controller.completed_kinds, set())

    def test_repeated_timestamp_stops(self):
        f = SensorFixture([surface()])
        f.until_traverse()
        result = f.controller.step(Observation(f.t, f.pose, f.grid))
        self.assertFalse(result.moving)
        self.assertEqual(f.controller.waypoint_index, 0)

    def test_progress_timeout_stops_stationary_robot(self):
        f = SensorFixture([surface()], Policy(validated_skills=KINDS,progress_timeout=1.))
        f.tick(start=True)
        for _ in range(12):
            f.tick()
        self.assertEqual(f.controller.phase, 'FAULT')
        self.assertFalse(f.last.moving)
        self.assertEqual(f.controller.completed_kinds, set())

    def test_ordered_surface_waypoints_cannot_be_skipped(self):
        f = SensorFixture([surface('slalom')])
        f.until_traverse()
        c = f.controller
        while c.waypoint_index == 0:
            f.advance_to(c.current.path[0])
            f.tick()
        # Обход обязательного изгиба: следующий изгиб достигается первым.
        goal = c.current.path[2]
        for _ in range(60):
            f.advance_to(goal)
            result = f.tick()
            if result.phase == 'FAULT':
                break
        self.assertEqual(c.phase, 'FAULT')
        self.assertFalse(c.completed_kinds)

    def test_timer_alone_cannot_verify_exit(self):
        f = SensorFixture([surface()])
        f.until_traverse()
        for _ in range(70):
            f.tick()
        self.assertEqual(f.controller.phase, 'FAULT')
        self.assertFalse(f.controller.completed_ids)

    def test_flat_bridge_follows_actual_forward_commands(self):
        f = SensorFixture([surface()])
        f.tick(start=True)
        distance = 0.
        for _ in range(1200):
            command = f.last
            yaw = f.pose.yaw + command.wz * .2
            dx,dy = command.vx * math.cos(yaw) * .2, command.vx * math.sin(yaw) * .2
            distance += math.hypot(dx,dy)
            f.pose = Pose(f.pose.x+dx,f.pose.y+dy,yaw,f.pose.z)
            f.tick()
            if f.controller.completed_kinds or f.controller.phase == 'FAULT':
                break
        self.assertEqual(f.controller.completed_kinds, {'bridge'}, f.last)
        self.assertGreater(distance,1.5)

    def test_raised_surface_uses_own_support_and_exits_full_footprint_to_floor(self):
        f = self.raised_fixture()
        f.tick(start=True)
        original_exit = f.items[0].path[-1][0]
        entered_raised = False
        for _ in range(1500):
            command = f.last
            yaw = f.pose.yaw+command.wz*.2
            f.pose = Pose(f.pose.x+command.vx*math.cos(yaw)*.2,
                          f.pose.y+command.vx*math.sin(yaw)*.2,yaw,7.4)
            # Независимый профиль синтетической поверхности, не целевой z FSM.
            f.support = max(0.,.12*(1-abs(f.pose.x-1.6)/.4)) if abs(f.pose.y) <= .75 else 0.
            entered_raised |= f.controller.phase == 'TRAVERSE' and f.pose.x > 1.5
            f.tick()
            if f.controller.completed_kinds or f.controller.phase == 'FAULT':
                break
        self.assertTrue(entered_raised)
        self.assertEqual(f.controller.completed_kinds,{'aframe'},f.last)
        self.assertGreater(f.pose.x,original_exit+.5)

    def test_surface_grid_cannot_clear_unknown_or_another_obstacle(self):
        for forbidden in (-1,1):
            with self.subTest(forbidden=forbidden):
                f = self.raised_fixture()
                f.grid.cells[8][8] = forbidden
                f.tick(start=True)
                f.tick()
                self.assertIsNone(f.controller.current)
                self.assertEqual(f.last.phase,'DISCOVER')
                self.assertIsNone(f.last.target_id)

    def test_raised_surface_needs_turning_footprint_width(self):
        f = self.raised_fixture()
        f.items = [replace(f.items[0],width=.8)]
        f.tick(start=True)
        f.tick()
        self.assertIsNone(f.controller.current)

    def test_return_requires_new_visible_crossing(self):
        f = SensorFixture()
        f.tick(start=True)
        c = f.controller
        c.completed_kinds = set(KINDS)
        c.completion_order = list(KINDS)
        self.assertEqual(f.tick(start_line_crossed=True).phase,'RETURN')
        self.assertEqual(f.tick(start_line_crossed=True).phase,'RETURN')
        f.tick(start_line_crossed=False)
        self.assertEqual(f.tick(start_line_crossed=True,start_line_visible=False).phase,'RETURN')
        self.assertEqual(f.tick(start_line_crossed=True).phase,'RETURN')
        f.tick(start_line_crossed=False)
        self.assertEqual(f.tick(start_line_crossed=True).phase,'RETURN')
        # Настоящий выход на сторону поля и новое пересечение к стороне старта.
        for _ in range(30):
            f.advance_to((.9,.125))
            f.tick()
            if f.pose.x >= .89:
                break
        for _ in range(30):
            f.advance_to((-.15,.125))
            f.tick()
            if c.phase == 'FINISHED':
                break
        self.assertEqual(c.phase,'FINISHED',f.last)
        self.assertFalse(f.last.moving)

    def test_return_without_geometry_waits_even_if_crossing_boolean_true(self):
        f = SensorFixture()
        f.tick(start=True,start_line=None)
        c = f.controller
        c.completed_kinds = set(KINDS)
        f.tick(start_line=None)
        self.assertFalse(f.tick(start_line=None,start_line_crossed=True).moving)
        self.assertEqual(c.phase,'RETURN')

    def test_return_geometry_quality_epoch_and_age_gates(self):
        changes = [dict(geometry_verified=False),dict(confirmations=2),dict(confidence=float('nan')),
            dict(confidence=.79),dict(frame_epoch=2),dict(observed_at=-10.),
            dict(endpoints=((.375,0.),(.375,.2)))]
        for change in changes:
            with self.subTest(change=change):
                f = SensorFixture()
                line = StartLine(f.line_endpoints,0.,confidence=1.,geometry_verified=True,confirmations=4)
                line = replace(line,**change)
                f.tick(start=True,start_line=line)
                f.controller.completed_kinds = set(KINDS)
                f.tick(start_line=line)
                result = f.tick(start_line=line,start_line_crossed=True)
                self.assertEqual(result.phase,'RETURN')
                self.assertFalse(result.moving)

    def test_return_accepts_valid_finite_line_crossing_far_from_start_pose(self):
        f = SensorFixture()
        f.tick(start=True)
        c = f.controller
        c.completed_kinds = set(KINDS)
        f.tick()
        # Одна и та же линия пересекается в другой точке, дальше старого
        # радиуса 0.475 м вокруг записанной стартовой позы.
        for goal in ((.9,.125),(.9,1.1),(.2,1.1)):
            for _ in range(40):
                f.advance_to(goal)
                f.tick()
                if c.phase == 'FINISHED' or math.dist((f.pose.x,f.pose.y),goal) < .001:
                    break
            if c.phase == 'FINISHED':
                break
        self.assertEqual(c.phase,'FINISHED',f.last)
        self.assertGreater(math.dist((f.pose.x,f.pose.y),(c.start_pose.x,c.start_pose.y)),.475)

    def test_crossing_extension_outside_finite_line_is_not_finish(self):
        f = SensorFixture()
        f.tick(start=True)
        c = f.controller
        c.completed_kinds = set(KINDS)
        f.tick()
        for goal in ((.9,.125),(.9,1.7),(.2,1.7)):
            for _ in range(40):
                f.advance_to(goal)
                # Даже ложное внешнее событие не заменяет конечный отрезок.
                crosses = f._observed_pose.x > .375 >= f.pose.x
                f.tick(start_line_crossed=crosses)
                self.assertNotEqual(c.phase,'FINISHED')
                if math.dist((f.pose.x,f.pose.y),goal) < .001:
                    break

    def test_return_plans_another_free_part_of_line_with_actual_commands(self):
        f = SensorFixture()
        f.pose = Pose(1.5,.125,math.pi)
        # Прямая проекция старта закрыта. Другой участок того же отрезка
        # свободен; движение определяется только командами контроллера.
        f.grid.cells[16][17] = 1
        f.tick(start=True)
        c = f.controller
        c.completed_kinds = set(KINDS)
        f.tick()
        for _ in range(1000):
            command = f.last
            yaw = f.pose.yaw+command.wz*.2
            f.pose = Pose(f.pose.x+command.vx*math.cos(yaw)*.2,
                          f.pose.y+command.vx*math.sin(yaw)*.2,yaw)
            f.tick()
            if c.phase in ('FINISHED','FAULT'):
                break
        self.assertEqual(c.phase,'FINISHED',f.last)
        self.assertGreater(abs(f.pose.y-c.start_pose.y),.5)

    def test_verification_requires_continuous_valid_observations(self):
        f = SensorFixture([surface()])
        f.until_traverse()
        for _ in range(200):
            c = f.controller
            if c.phase == 'VERIFY':
                break
            f.advance_to(c.current.path[c.waypoint_index])
            f.tick()
        self.assertEqual(c.phase,'VERIFY')
        f.tick()
        f.tick()
        self.assertEqual(c._verify_count,2)
        f.tick(localized=False)
        self.assertEqual(c._verify_count,0)
        self.assertIsNone(c._verify_t)
        for _ in range(5):
            f.tick(localized=False)
        f.tick()
        self.assertEqual(c.phase,'VERIFY')
        self.assertFalse(c.completed_kinds)

    def test_unreachable_near_duplicate_does_not_hide_reachable_kind(self):
        near = surface(origin=(-2.2,.125), name='near-behind-wall')
        far = surface(origin=(3.8,.125), name='far-accessible')
        f = SensorFixture([near,far])
        f.pose = Pose(.625,.125,0.)
        # Стена полностью делит карту на независимые области.
        for row in f.grid.cells:
            row[14] = 1
        f.tick(start=True)
        f.tick()
        self.assertEqual(f.controller.current.id,'far-accessible')

    def test_reverse_permission_revocation_stops(self):
        item = replace(surface(origin=(-2.,.125)),bidirectional=True)
        f = SensorFixture([item])
        f.pose = Pose(.125,.125,math.pi)
        f.tick(start=True)
        f.tick()
        # Состояние ранее выбранного разрешённого обратного прохождения.
        # Сам выбор направления отдельно проверяется тестами оптимизатора.
        c = f.controller
        self.assertIsNotNone(c.current)
        if not c._reverse:
            c.current = replace(c.current,path=tuple(reversed(c.current.path)))
            c._reverse = True
        f.items = [replace(item,bidirectional=False)]
        self.assertFalse(f.tick().moving)
        self.assertIn('обратного',f.last.reason)

    def test_invalid_identity_does_not_crash(self):
        f = SensorFixture([replace(surface(),id=['unhashable'])])
        f.tick(start=True)
        self.assertFalse(f.tick().moving)
        self.assertFalse(f.controller.memory)

    def test_all_five_shuffled_observed_locations_exactly_once_and_return(self):
        self.check_full_course(KINDS)

    def test_physical_four_ignore_extra_bridge_and_return(self):
        self.check_full_course(('slalom','aframe','teeter','platforms'))

    def test_physical_four_return_to_point_without_line(self):
        self.check_full_course(('slalom','aframe','teeter','platforms'),return_mode='point')

    def test_home_confirmation_resets_after_bad_observation(self):
        f=SensorFixture(policy=Policy(validated_skills=KINDS))
        f.tick(start=True,start_line_visible=False)
        c=f.controller
        c.completed_kinds=set(f.policy.required_kinds)
        f.tick()
        f.tick()
        self.assertIsNotNone(c._home_stable_since)
        f.tick(localized=False)
        self.assertIsNone(c._home_stable_since)
        for _ in range(3):self.assertNotEqual(f.tick().phase,'FINISHED')

    def test_point_return_requires_stopped_robot_and_support(self):
        for change in ({'speed':.1},{'support_verified':False}):
            f=SensorFixture(policy=Policy(validated_skills=KINDS))
            f.tick(start=True,start_line_visible=False)
            c=f.controller;c.completed_kinds=set(f.policy.required_kinds)
            f.tick()
            for _ in range(20):
                self.assertNotEqual(f.tick(**change).phase,'FINISHED')
            self.assertIsNone(c._home_stable_since)

    def check_full_course(self,required,return_mode='line'):
        items = [surface('bridge', origin=(1.2,0.)), surface('aframe',origin=(3.,1.),yaw=math.pi/2),
            surface('platforms',origin=(0.,3.),yaw=math.pi),
            surface('slalom',origin=(-2.,1.5),yaw=-math.pi/2),
            surface('teeter',origin=(0.,-2.),yaw=0.),
            surface('bridge',name='bridge-second',origin=(4.,-1.))]
        random.Random(170926).shuffle(items)
        f = SensorFixture(items,Policy(validated_skills=KINDS,required_kinds=required,return_mode=return_mode,max_mission_seconds=1200.))
        c = f.controller
        phases, counted_paths = set(), {}
        f.tick(start=True,start_line_visible=return_mode=='line')
        for _ in range(2500):
            phases.add(c.phase)
            if c.phase in ('APPROACH','RETURN'):
                goal = c._approach_goal if c.phase == 'APPROACH' else c._return_goal or (c.start_pose.x,c.start_pose.y)
                if c._nav_path and c._nav_index < len(c._nav_path):
                    goal = c._nav_path[c._nav_index]
                f.advance_to(goal)
            elif c.phase == 'ALIGN':
                a,b = c.current.path[:2]
                f.advance_to((f.pose.x,f.pose.y),math.atan2(b[1]-a[1],b[0]-a[0]))
            elif c.phase == 'TRAVERSE':
                f.advance_to(c.current.path[c.waypoint_index])
            elif c.phase == 'VERIFY':
                counted_paths[c.current.kind] = [v[0] for v in c.visited_waypoints]
            f.tick(start_line_visible=return_mode=='line')
            if c.phase in ('FINISHED','FAULT'):
                break
        self.assertEqual(c.phase,'FINISHED',f.last)
        self.assertEqual(c.completed_kinds,set(required))
        self.assertEqual(len(c.completed_ids),len(required))
        self.assertEqual(len(c.completion_order),len(required))
        self.assertEqual(counted_paths['platforms'],list(range(5)))
        self.assertEqual(counted_paths['slalom'],list(range(5)))
        self.assertTrue({'DISCOVER','APPROACH','ALIGN','TRAVERSE','VERIFY','RETURN'} <= phases)
        self.assertFalse(f.last.moving)
        if return_mode=='point':
            self.assertLessEqual(math.dist((f.pose.x,f.pose.y),(c.start_pose.x,c.start_pose.y))+.02,f.policy.return_radius)


if __name__ == '__main__':
    unittest.main()
