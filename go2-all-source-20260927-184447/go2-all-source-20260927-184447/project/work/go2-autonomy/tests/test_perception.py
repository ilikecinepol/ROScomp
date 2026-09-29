"""Проверки наблюдаемой геометрии; синтетика не подтверждает проходимость Go2."""
import math
import unittest
from pathlib import Path

import numpy as np

from wolf_go2.models import Pose, Policy
from wolf_go2.perception import PerceptionPipeline, adapter_observe, camera_candidates


def contract(**extra):
    value = dict(geometry_frames_validated=True, source_time_validated=True, identity_verified=True,
                 units='m', up_axis='+z', point_frame='verified_world', pose_frame='verified_world',
                 pose_child_frame='base_link', source_kind='observed_surface_points', acquisition_age_s=.02,
                 localization_error_m=.01, surface_resolution_m=.05)
    value.update(extra)
    return value


def scene(kind='floor', width=1., holes=False):
    x, y = np.meshgrid(np.arange(-2., 7.01, .05), np.arange(-2.5, 2.51, .05))
    z = np.zeros_like(x)
    on = (x >= 1.-1e-8) & (x <= 3.+1e-8) & (abs(y) <= width/2+1e-8)
    if kind == 'aframe':
        z[on] = .3*np.maximum(0., 1.-abs(x[on]-2.))
    elif kind == 'platforms':
        z[on] = np.where(x[on] < 1.7, .08, np.where(x[on] <= 2.3, .16, .16*(3.-x[on])/.7))
    elif kind == 'deck':
        z[on] = .15
    points = np.column_stack([x.ravel(), y.ravel(), z.ravel()])
    if holes:
        points = points[~((abs(points[:, 0]) < .2) & (abs(points[:, 1]) < .2))]
    return points


def cone_scene():
    points = [scene()]
    for cx in (1., 3., 5.):
        for z in np.arange(.04, .401, .03):
            angles = np.linspace(0, 2*math.pi, 48, endpoint=False)
            radius = .16*(1-z/.44)
            points.append(np.column_stack([cx+radius*np.cos(angles), radius*np.sin(angles), np.full(len(angles), z)]))
    return np.concatenate(points)


def confirm(engine, points, geometry=None, pose=Pose(0., 0., 0., .3), epoch=0):
    for t in (1., 1.1, 1.2):
        result = engine.build_observation(t, pose, points, epoch, contract() if geometry is None else geometry)
    return result


class PerceptionTests(unittest.TestCase):
    def test_missing_contract_and_nonfinite_pose_block(self):
        engine = PerceptionPipeline()
        self.assertFalse(engine.build_observation(1., Pose(0, 0, 0), scene()).localized)
        for changes in ({'identity_verified': False}, {'source_time_validated': False}, {'pose_frame': 'elsewhere'},
                        {'acquisition_age_s': .7}, {'pose_child_frame': ''}, {'localization_error_m': float('nan')}, {'frame_epoch': 8}):
            observation = engine.build_observation(1., Pose(0, 0, 0), scene(), geometry_contract=contract(**changes))
            self.assertFalse(observation.localized, changes)
            self.assertIsNone(observation.grid)
        self.assertFalse(engine.build_observation(1., Pose(float('nan'), 0, 0), scene(), geometry_contract=contract()).localized)

    def test_floor_and_unknown_hole(self):
        normal = confirm(PerceptionPipeline(), scene())
        self.assertTrue(normal.localized)
        self.assertTrue(normal.support_verified)
        self.assertAlmostEqual(normal.support_height_m, 0.)
        self.assertEqual(normal.obstacles, [])
        hole = confirm(PerceptionPipeline(), scene(holes=True))
        self.assertFalse(hole.support_verified)
        ix = int((0-hole.grid.origin[0])/hole.grid.resolution)
        iy = int((0-hole.grid.origin[1])/hole.grid.resolution)
        self.assertEqual(hole.grid.cells[iy][ix], -1)

    def test_aframe_tracking_and_epoch(self):
        engine = PerceptionPipeline()
        observation = confirm(engine, scene('aframe'))
        self.assertEqual(len(observation.obstacles), 1, engine.candidates)
        obstacle = observation.obstacles[0]
        self.assertEqual(obstacle.kind, 'aframe')
        self.assertTrue(obstacle.geometry_verified, engine.candidates)
        self.assertGreater(obstacle.width, .9)
        self.assertAlmostEqual(obstacle.max_slope_deg, math.degrees(math.atan(.3)), delta=1.)
        self.assertTrue(any(p[2] > .28 for p in obstacle.path))
        self.assertLess(obstacle.path[0][0], 1.)
        self.assertGreater(obstacle.path[-1][0], 3.)
        duplicate = engine.build_observation(1.2, Pose(0, 0, 0, .3), scene('aframe'), geometry_contract=contract())
        self.assertEqual(duplicate.obstacles[0].confirmations, 3)
        reset = engine.build_observation(1.3, Pose(0, 0, 0, .3), scene('aframe'), frame_epoch=1, geometry_contract=contract())
        self.assertNotEqual(reset.obstacles[0].id, obstacle.id)
        self.assertEqual(reset.obstacles[0].confirmations, 1)
        self.assertFalse(reset.obstacles[0].geometry_verified)

    def test_platforms_two_rises_derived(self):
        engine = PerceptionPipeline()
        observation = confirm(engine, scene('platforms'))
        self.assertEqual(len(observation.obstacles), 1, engine.candidates)
        obstacle = observation.obstacles[0]
        self.assertEqual(obstacle.kind, 'platforms')
        self.assertTrue(obstacle.geometry_verified, engine.candidates)
        self.assertAlmostEqual(obstacle.max_step_m, .08, places=5)
        self.assertFalse(obstacle.bidirectional)
        self.assertTrue(obstacle.direction_verified)
        self.assertGreater(len(set(round(p[2], 2) for p in obstacle.path)), 2)

    def test_platform_direction_is_geometry_not_robot_side(self):
        points = scene('platforms')
        forward = confirm(PerceptionPipeline(), points).obstacles[0]
        reverse_view = confirm(PerceptionPipeline(), points, pose=Pose(4., 0., math.pi, .3)).obstacles[0]
        self.assertTrue(reverse_view.geometry_verified)
        self.assertFalse(reverse_view.bidirectional)
        self.assertLess(reverse_view.path[0][0], reverse_view.path[-1][0])
        np.testing.assert_allclose(reverse_view.path, forward.path, atol=1e-7)

    def test_two_steps_without_ramp_do_not_claim_direction(self):
        points = scene('platforms')
        on = (points[:, 0] >= 1.) & (points[:, 0] <= 3.01) & (abs(points[:, 1]) <= .5+1e-8)
        points[on, 2] = np.where((points[on, 0] < 1.7) | (points[on, 0] > 2.3), .08, .16)
        engine = PerceptionPipeline()
        observation = confirm(engine, points)
        self.assertFalse(any(item.direction_verified for item in observation.obstacles), engine.candidates)

    def test_flat_deck_has_no_unjustified_mechanism(self):
        engine = PerceptionPipeline()
        observation = confirm(engine, scene('deck'))
        self.assertEqual(observation.obstacles, [])
        unknown = [item for item in engine.candidates if item['kind'] == 'unknown']
        self.assertTrue(unknown)
        self.assertTrue(any('шарнир' in reason for reason in unknown[0]['reasons']))
        manual = contract(structure_evidence={'kind': 'bridge', 'verified': True, 'center_xy': [2., 0.],
                          'verification_id': 'manual', 'rigid_supports_verified': True, 'ends_supported_verified': True})
        self.assertEqual(confirm(engine, scene('deck'), manual).obstacles, [])

    def test_narrow_or_split_ramp_not_verified(self):
        for points in (scene('aframe', width=.4), scene('aframe')[abs(scene('aframe')[:, 1]) > .10]):
            engine = PerceptionPipeline()
            observation = confirm(engine, points)
            self.assertFalse(any(item.geometry_verified for item in observation.obstacles), engine.candidates)

    def test_slalom_has_alternating_supported_waypoints(self):
        engine = PerceptionPipeline()
        observation = confirm(engine, cone_scene(), contract(slalom_first_side='left'))
        slalom = [obstacle for obstacle in observation.obstacles if obstacle.kind == 'slalom']
        self.assertEqual(len(slalom), 1, engine.candidates)
        self.assertTrue(slalom[0].geometry_verified, engine.candidates)
        candidate = next(item for item in engine.candidates if item['kind'] == 'slalom')
        sides = [np.sign(point[1]) for point in candidate['alternating_waypoints_xy']]
        self.assertEqual(sides, [1., -1., 1.])
        self.assertGreater(len(slalom[0].path), 10)
        self.assertFalse(slalom[0].bidirectional)
        no_side = confirm(PerceptionPipeline(), cone_scene())
        self.assertTrue(any(item.geometry_verified for item in no_side.obstacles))

    def test_tracked_paths_do_not_flip_when_robot_passes_center(self):
        for points, geometry, kind in ((scene('aframe'), contract(), 'aframe'),
                                       (cone_scene(), contract(slalom_first_side='left'), 'slalom')):
            engine = PerceptionPipeline()
            before = confirm(engine, points, geometry)
            old = next(item for item in before.obstacles if item.kind == kind)
            after = engine.build_observation(1.3, Pose(6., 0., math.pi, .3), points, geometry_contract=geometry)
            new = next(item for item in after.obstacles if item.kind == kind)
            self.assertEqual(old.id, new.id)
            np.testing.assert_allclose(new.path, old.path, atol=1e-7)

    def test_elevated_support_measured_from_current_points(self):
        engine = PerceptionPipeline()
        observation = confirm(engine, scene('aframe', width=1.3), pose=Pose(1.5, 0., 0., .45))
        self.assertTrue(observation.support_verified, observation.diagnostics)
        self.assertAlmostEqual(observation.support_height_m, .15, places=6)
        deck_and_underfloor = np.concatenate([scene('deck', width=1.3), scene()])
        observation = confirm(PerceptionPipeline(), deck_and_underfloor, pose=Pose(2., 0., 0., .45))
        self.assertTrue(observation.support_verified)
        self.assertAlmostEqual(observation.support_height_m, .15, places=6)
        hole = deck_and_underfloor[~((abs(deck_and_underfloor[:, 0]-2.) < .1) &
                                    (abs(deck_and_underfloor[:, 1]) < .1) & (deck_and_underfloor[:, 2] > .1))]
        observation = confirm(PerceptionPipeline(), hole, pose=Pose(2., 0., 0., .45))
        self.assertFalse(observation.support_verified)

    def test_body_height_is_only_crosscheck_with_verified_reference(self):
        points = scene('deck', width=1.3)
        supplied_but_unverified = contract(body_height_m=.3)
        observation = confirm(PerceptionPipeline(), points, supplied_but_unverified, pose=Pose(2., 0., 0., .6))
        self.assertAlmostEqual(observation.support_height_m, .15, places=6)
        checked = contract(body_height_m=.3, body_height_reference_verified=True, body_height_reference_evidence='synthetic known frame')
        observation = confirm(PerceptionPipeline(), points, checked, pose=Pose(2., 0., 0., .6))
        self.assertFalse(observation.support_verified)

    def test_per_object_surface_grid_preserves_unknown_and_other_objects(self):
        points = scene('aframe', width=1.4)
        points = points[~((abs(points[:, 0]+1.4) < .2) & (abs(points[:, 1]+1.5) < .2))]
        # Второй предмет за пределами поверхности ската не должен очищаться.
        wall = (points[:, 0] > 5.) & (points[:, 0] < 5.3) & (abs(points[:, 1]) < .2)
        points[wall, 2] = .5
        observation = confirm(PerceptionPipeline(), points)
        ramp = next(item for item in observation.obstacles if item.kind == 'aframe')
        self.assertTrue(ramp.geometry_verified)
        original = np.asarray(observation.grid.cells)
        traverse = np.asarray(observation.traversal_grids[ramp.id].cells)
        self.assertTrue(np.array_equal(traverse[original == -1], original[original == -1]))
        self.assertTrue(np.any((original == 1) & (traverse == 0)))
        self.assertTrue(np.any(traverse == 1))
        from wolf_go2.navigation import path_is_clear
        self.assertFalse(path_is_clear(observation.grid, [p[:2] for p in ramp.path], Policy(), .01))
        self.assertTrue(path_is_clear(observation.traversal_grids[ramp.id], [p[:2] for p in ramp.path], Policy(), .01))

    def test_cloud_perception_to_mission_accepts_observed_raised_surface(self):
        from wolf_go2.mission import MissionController
        from wolf_go2.models import StartLine
        policy = Policy(validated_skills=('aframe',))
        engine = PerceptionPipeline()
        points = scene('aframe', width=1.4)
        observation = confirm(engine, points)
        observation.body_height = .3
        observation.start_line_visible = True
        observation.start_line = StartLine(((.25, -1.), (.25, 1.)), observation.t,
                                           confidence=1., geometry_verified=True, confirmations=3)
        controller = MissionController(policy)
        ramp = observation.obstacles[0]
        self.assertEqual(controller._candidate_errors(ramp, observation), [])
        self.assertEqual(controller.step(observation, start=True).phase, 'DISCOVER')
        next_observation = engine.build_observation(1.3, Pose(0., 0., 0., .3), points, geometry_contract=contract(), policy=policy)
        next_observation.body_height = .3
        decision = controller.step(next_observation)
        self.assertEqual(decision.phase, 'APPROACH', decision.reason)
        self.assertEqual(controller.current.id, ramp.id)
        # Отдельная проверка наблюдения на скате, без имитации фактического шага робота.
        on_ramp = engine.build_observation(1.4, Pose(1.5, 0., 0., .45), points, geometry_contract=contract(), policy=policy)
        on_ramp.body_height = .3
        controller.phase = 'TRAVERSE'
        self.assertEqual(controller._pose_errors(on_ramp), [])
        self.assertTrue(on_ramp.support_verified)
        self.assertAlmostEqual(on_ramp.support_height_m, .15, places=6)

    def test_camera_bbox_never_becomes_metric_target(self):
        image = np.zeros((180, 240, 3), dtype=np.uint8)
        image[30:110, 95:140] = (0, 140, 255)
        candidates = camera_candidates(image)
        cone = next(item for item in candidates if item['kind'] == 'cone_image_candidate')
        self.assertFalse(cone['metric_target'])
        result = adapter_observe({'t': 1., 'pose': Pose(0, 0, 0), 'points': scene(), 'image': image})
        self.assertEqual(result.obstacles, [])
        self.assertFalse(result.localized)

    def test_voxel_anchor_requires_explicit_geometry(self):
        points = scene()
        points[:, 2] = -.05
        engine = PerceptionPipeline()
        missing = contract(source_kind='occupied_voxel_points')
        self.assertFalse(confirm(engine, points, missing).localized)
        valid = contract(source_kind='occupied_voxel_points', voxel_reference='lower_corner', voxel_size_m=.05)
        surface = confirm(PerceptionPipeline(), points, valid)
        self.assertTrue(surface.localized)
        self.assertTrue(surface.support_verified)
        self.assertAlmostEqual(surface.support_height_m, 0., places=6)

    def test_weak_frames_do_not_confirm_strong_geometry(self):
        engine = PerceptionPipeline()
        confirm(engine, scene('aframe', width=.4))
        observation = engine.build_observation(1.3, Pose(0, 0, 0, .3), scene('aframe'), geometry_contract=contract())
        self.assertEqual(observation.obstacles[0].confirmations, 1)
        self.assertFalse(observation.obstacles[0].geometry_verified)
        backward = engine.build_observation(1.1, Pose(0, 0, 0, .3), scene('aframe'), geometry_contract=contract())
        self.assertFalse(backward.localized)

    def test_cached_cloud_does_not_count_as_three_measurements(self):
        engine = PerceptionPipeline()
        points = scene('aframe')
        for stamp in (1., 1.1, 1.2):
            result = engine.build_observation(stamp, Pose(0, 0, 0, .3), points,
                geometry_contract=contract(acquisition_age_s=stamp-.98))
        self.assertEqual(result.obstacles[0].confirmations, 1)
        self.assertFalse(result.obstacles[0].geometry_verified)
        self.assertAlmostEqual(result.obstacles[0].observed_at, .98)

    def test_aframe_measures_lip_instead_of_extrapolating_it_away(self):
        points = scene('aframe')
        raised = points[:, 2] > .001
        points[raised, 2] += .08
        engine = PerceptionPipeline()
        observation = confirm(engine, points)
        self.assertEqual(len(observation.obstacles), 1, engine.candidates)
        self.assertGreaterEqual(observation.obstacles[0].max_lip_m, .07)

    def test_cylinders_are_not_confirmed_cones(self):
        points = [scene()]
        for cx in (1., 3., 5.):
            for z in np.arange(.05, .41, .04):
                angles = np.linspace(0, 2*math.pi, 48, endpoint=False)
                points.append(np.column_stack([cx+.15*np.cos(angles), .15*np.sin(angles), np.full(len(angles), z)]))
        observation = confirm(PerceptionPipeline(), np.concatenate(points), contract(slalom_first_side='left'))
        self.assertFalse(any(item.kind == 'slalom' for item in observation.obstacles))

    def test_ridged_pyramid_is_not_a_flat_ramp(self):
        points = scene('aframe')
        raised = points[:, 2] > .001
        points[raised, 2] *= np.maximum(0., 1.-abs(points[raised, 1])/0.55)
        engine = PerceptionPipeline()
        result = confirm(engine, points)
        self.assertFalse(any(item.geometry_verified for item in result.obstacles), engine.candidates)

    def test_five_saved_npz_block_without_contract(self):
        # Данные необязательны для переносимого набора тестов, но используются здесь.
        root = Path(__file__).resolve().parents[3]
        files = sorted((root/'outputs/go2-remote-setup/logs/20260916T112503Z').glob('lidar_*.npz'))
        if not files:
            self.skipTest('Локальная запись отсутствует')
        self.assertEqual(len(files), 5)
        for path in files:
            with np.load(path, allow_pickle=False) as archive:
                keys = [key for key in archive.files if key.endswith('points')]
                self.assertEqual(len(keys), 1)
                points = archive[keys[0]]
            result = PerceptionPipeline().build_observation(1., Pose(0, 0, 0), points)
            self.assertFalse(result.localized, path.name)
            self.assertIsNone(result.grid)
            self.assertEqual(result.obstacles, [])


if __name__ == '__main__':
    unittest.main()
