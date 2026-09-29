"""Синтетические проверки геометрии; физических гарантий роботу не дают."""
import math
import random
import unittest
from copy import deepcopy
from dataclasses import replace

from wolf_go2.models import Grid, Obstacle, Policy, Pose
from wolf_go2.navigation import (astar, follow_path, frontier_path, optimize_route,
                                 path_is_clear, safe_mask)


def grid(width=50, height=40, resolution=.2):
    return Grid((0., 0.), resolution, [[0] * width for _ in range(height)], verified=True)


def point(x, y):
    return ((x + .5) * .2, (y + .5) * .2)


def obstacle(name, start, end, **changes):
    return Obstacle(name, 'bridge', ((*start, 0.), (*end, 0.)), .8, 0.,
                    confidence=1., confirmations=3, geometry_verified=True,
                    direction_verified=True, **changes)


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.grid = grid()
        self.policy = Policy(robot_length=.20, robot_width=.16, clearance=.02)

    def test_unverified_grid_blocks_all_apis(self):
        self.grid.verified = False
        self.assertFalse(any(map(any, safe_mask(self.grid, self.policy, 0.))))
        self.assertIsNone(astar(self.grid, point(5, 5), point(20, 5), self.policy, 0.))
        self.assertFalse(path_is_clear(self.grid, [point(5, 5)], self.policy, 0.))

    def test_unknown_is_not_free(self):
        self.grid.cells[15][15] = -1
        mask = safe_mask(self.grid, self.policy, 0.)
        self.assertFalse(mask[15][15])
        self.assertFalse(mask[15][14])
        self.assertTrue(mask[15][12])
        self.assertIsNone(astar(self.grid, point(5, 5), point(15, 15), self.policy, 0.))

    def test_border_and_outside_blocked(self):
        mask = safe_mask(self.grid, self.policy, 0.)
        self.assertFalse(mask[0][20])
        self.assertFalse(mask[-1][20])
        self.assertTrue(mask[2][20])
        self.assertIsNone(astar(self.grid, (-.01, 2.), point(5, 5), self.policy, 0.))

    def test_inflation_uses_length_as_well_as_width(self):
        for y in range(40):
            self.grid.cells[y][10] = 1
            self.grid.cells[y][14] = 1
        small = safe_mask(self.grid, self.policy, 0.)
        long = safe_mask(self.grid, replace(self.policy, robot_length=.8), 0.)
        self.assertTrue(small[20][12])
        self.assertFalse(long[20][12])

    def test_uncertainty_expands_mask_and_excess_blocks(self):
        self.grid.cells[15][15] = 1
        small = safe_mask(self.grid, self.policy, 0.)
        larger = safe_mask(self.grid, self.policy, .08)
        self.assertGreater(sum(map(sum, small)), sum(map(sum, larger)))
        self.assertFalse(any(map(any, safe_mask(self.grid, self.policy, .081))))
        self.assertFalse(any(map(any, safe_mask(self.grid, self.policy, float('nan')))))

    def test_inflation_matches_independent_rectangle_distances(self):
        rng = random.Random(170926)
        g = grid(22, 20)
        occupied = []
        for y in range(20):
            for x in range(22):
                if rng.random() < .035:
                    g.cells[y][x] = rng.choice((-1, 1))
                    occupied.append((x, y))
        for uncertainty in (0., .08):
            mask = safe_mask(g, self.policy, uncertainty)
            radius = math.hypot(self.policy.robot_length, self.policy.robot_width) / 2 + self.policy.clearance + uncertainty
            for y in range(20):
                for x in range(22):
                    border_ok = min(x, y, 21 - x, 19 - y) * .2 > radius + 1e-10
                    expected = border_ok and all(math.hypot(
                        max(0., (max(x, ox) - min(x + 1, ox + 1)) * .2),
                        max(0., (max(y, oy) - min(y + 1, oy + 1)) * .2)) > radius + 1e-10
                        for ox, oy in occupied)
                    self.assertEqual(mask[y][x], expected, (x, y, uncertainty))

    def test_malformed_grid_fails_closed(self):
        self.grid.cells[0].pop()
        self.assertEqual(safe_mask(self.grid, self.policy, 0.), [])
        self.assertIsNone(astar(self.grid, point(5, 5), point(20, 5), self.policy, 0.))

    def test_astar_keeps_endpoint_cells_and_returns_centers(self):
        start, goal = (1.04, 1.08), (5.19, 4.01)
        path = astar(self.grid, start, goal, self.policy, 0.)
        self.assertEqual(path[0], point(5, 5))
        self.assertEqual(path[-1], point(25, 20))
        self.assertTrue(path_is_clear(self.grid, [start] + path + [goal], self.policy, 0.))
        for a, b in zip(path, path[1:]):
            self.assertLessEqual(math.dist(a, b), math.sqrt(2) * .2 + 1e-10)

    def test_same_cell_is_validated(self):
        self.assertEqual(astar(self.grid, (1.05, 1.05), (1.06, 1.06), self.policy, 0.), [point(5, 5)])
        self.grid.cells[5][5] = 1
        self.assertIsNone(astar(self.grid, (1.05, 1.05), (1.06, 1.06), self.policy, 0.))

    def test_diagonal_touching_rooms_do_not_allow_corner_cut(self):
        g = grid(24, 24)
        g.cells = [[1] * 24 for _ in range(24)]
        for low, high in ((2, 12), (12, 22)):
            for y in range(low, high):
                for x in range(low, high):
                    g.cells[y][x] = 0
        self.assertIsNone(astar(g, point(5, 5), point(18, 18), self.policy, 0.))

    def test_astar_detours_and_segment_check_rejects_wall(self):
        for y in range(8, 24):
            self.grid.cells[y][20] = 1
        start, goal = point(10, 16), point(30, 16)
        self.assertFalse(path_is_clear(self.grid, [start, goal], self.policy, 0.))
        path = astar(self.grid, start, goal, self.policy, 0.)
        self.assertIsNotNone(path)
        self.assertTrue(path_is_clear(self.grid, path, self.policy, 0.))
        self.assertGreater(sum(math.dist(a, b) for a, b in zip(path, path[1:])), math.dist(start, goal))

    def test_continuous_segment_cannot_miss_small_corner(self):
        self.grid.cells[15][15] = 1
        self.assertFalse(path_is_clear(self.grid, [point(5, 5), point(25, 25)], self.policy, 0.))
        self.assertFalse(path_is_clear(self.grid, [], self.policy, 0.))
        self.assertFalse(path_is_clear(self.grid, [(math.nan, 1.)], self.policy, 0.))

    def test_follow_first_waypoint_not_later_shortcut(self):
        pose = Pose(2., 2., math.pi / 2)
        vx, wz = follow_path(pose, [(2., 2.), (4., 2.), (4., 4.)], self.policy)
        self.assertEqual(vx, 0.)
        self.assertLess(wz, 0.)
        self.assertLessEqual(abs(wz), self.policy.max_wz)

    def test_follower_turns_before_backward_target(self):
        vx, wz = follow_path(Pose(2., 2., 0.), [(1., 2.)], self.policy)
        self.assertEqual(vx, 0.)
        self.assertGreater(abs(wz), 0.)

    def test_follower_brakes_and_respects_limit(self):
        far = follow_path(Pose(2., 2., 0.), [(4., 2.)], self.policy)[0]
        near = follow_path(Pose(3.88, 2., 0.), [(4., 2.)], self.policy)[0]
        limited = follow_path(Pose(2., 2., 0.), [(4., 2.)], self.policy, speed_limit=.07)[0]
        self.assertGreater(far, near)
        self.assertLessEqual(limited, .07)
        self.assertEqual(follow_path(Pose(4., 2., 0.), [(4., 2.)], self.policy), (0., 0.))

    def test_curvature_reduces_speed(self):
        straight = follow_path(Pose(2., 2., 0.), [(2.5, 2.)], self.policy, stop_at_end=False)[0]
        curve = follow_path(Pose(2., 2., .3), [(2.5, 2.)], self.policy, stop_at_end=False)[0]
        self.assertGreater(straight, curve)
        self.assertGreater(curve, 0.)

    def test_follower_invalid_input_stops(self):
        self.assertEqual(follow_path(Pose(2., 2., math.nan), [(4., 2.)], self.policy), (0., 0.))
        self.assertEqual(follow_path(Pose(2., 2., 0.), [(4., 2.)], self.policy, speed_limit=-1), (0., 0.))
        self.assertEqual(follow_path(Pose(2., 2., 0.), [(4., 2.)], self.policy, speed_limit=math.nan), (0., 0.))

    def test_frontier_stays_in_known_free_space(self):
        for y in range(40):
            for x in range(30, 50):
                self.grid.cells[y][x] = -1
        pose = Pose(*point(8, 18), 0.)
        path = frontier_path(self.grid, pose, self.policy, 0.)
        self.assertIsNotNone(path)
        self.assertGreater(path[-1][0], pose.x)
        self.assertLess(path[-1][0], 6.)
        self.assertTrue(path_is_clear(self.grid, path, self.policy, 0.))
        self.assertEqual(frontier_path(grid(), pose, self.policy, 0.), None)

    def test_frontier_behind_solid_wall_is_unreachable(self):
        for y in range(40):
            self.grid.cells[y][20] = 1
            for x in range(30, 50):
                self.grid.cells[y][x] = -1
        self.assertIsNone(frontier_path(self.grid, Pose(*point(8, 18), 0.), self.policy, 0.))

    def test_route_selects_near_entry_order_not_input_order(self):
        first = obstacle('first', (2., 3.), (3., 3.))
        last = obstacle('last', (6., 3.), (7., 3.))
        route = optimize_route(Pose(1., 3., 0.), [last, first], self.grid, self.policy, 0.)
        self.assertEqual(route, [('first', False), ('last', False)])

    def test_route_reverse_only_when_explicitly_bidirectional(self):
        ob = obstacle('ramp', (2., 3.), (5., 3.))
        pose = Pose(6., 3., math.pi)
        self.assertEqual(optimize_route(pose, [ob], self.grid, self.policy, 0.), [('ramp', False)])
        self.assertEqual(optimize_route(pose, [replace(ob, bidirectional=True)], self.grid, self.policy, 0.), [('ramp', True)])
        self.assertEqual(optimize_route(pose, [replace(ob, bidirectional=True, direction_verified=False)], self.grid, self.policy, 0.), [])

    def test_route_requires_surface_and_matching_coordinates(self):
        ob = obstacle('ramp', (2.1, 3.1), (5.1, 3.1))
        pose = Pose(1., 3., 0.)
        self.assertEqual(optimize_route(pose, [replace(ob, frame_epoch=1)], self.grid, self.policy, 0.), [])
        self.assertEqual(optimize_route(pose, [replace(ob, geometry_verified=False)], self.grid, self.policy, 0.), [])
        self.grid.cells[15][18] = -1
        self.assertEqual(optimize_route(pose, [ob], self.grid, self.policy, 0.), [])

    def test_route_requires_feasible_connection_to_every_obstacle(self):
        for y in range(40):
            self.grid.cells[y][24] = 1
        left = obstacle('left', (2., 3.), (3., 3.))
        right = obstacle('right', (6., 3.), (7., 3.))
        self.assertEqual(optimize_route(Pose(1., 3., 0.), [left, right], self.grid, self.policy, 0.), [])

    def test_route_home_must_be_reachable(self):
        ob = obstacle('bridge', (2., 3.), (3., 3.))
        pose = Pose(1., 3., 0.)
        self.assertEqual(optimize_route(pose, [ob], self.grid, self.policy, 0., home=pose), [('bridge', False)])
        self.assertEqual(optimize_route(pose, [ob], self.grid, self.policy, 0., home=Pose(-1., 3., 0.)), [])

    def test_five_goal_route_has_every_id_once(self):
        obstacles = [obstacle(str(i), (1.4 + i * 1.3, 3.), (2. + i * 1.3, 3.), bidirectional=True) for i in range(5)]
        route = optimize_route(Pose(1., 3., 0.), list(reversed(obstacles)), self.grid, self.policy, 0., home=Pose(1., 3., 0.))
        self.assertEqual(len(route), 5)
        self.assertEqual({name for name, _ in route}, {str(i) for i in range(5)})

    def test_two_raised_surfaces_use_separate_maps_and_floor_stages(self):
        first = replace(obstacle('first', (2.1, 3.1), (3.1, 3.1)),
                        path=((2.1, 3.1, 0.), (2.6, 3.1, .15), (3.1, 3.1, 0.)))
        second = replace(obstacle('second', (6.1, 3.1), (7.1, 3.1)),
                         path=((6.1, 3.1, 0.), (6.6, 3.1, .15), (7.1, 3.1, 0.)))
        for x in list(range(10, 16)) + list(range(30, 36)):
            self.grid.cells[15][x] = 1
        floor_before = deepcopy(self.grid.cells)
        maps = {name: deepcopy(self.grid) for name in ('first', 'second')}
        for name, columns in (('first', range(10, 16)), ('second', range(30, 36))):
            for x in columns:
                maps[name].cells[15][x] = 0
        stages = {('first', False): ((1.5, 3.1), (3.7, 3.1)),
                  ('second', False): ((5.5, 3.1), (7.7, 3.1))}
        pose = Pose(1.1, 3.1, 0.)
        self.assertEqual(optimize_route(pose, [first, second], self.grid, self.policy, 0.), [])
        route = optimize_route(pose, [second, first], self.grid, self.policy, 0.,
                               surface_grids=maps, stages=stages, home=pose)
        self.assertEqual(route, [('first', False), ('second', False)])
        self.assertTrue(path_is_clear(maps['first'], first.path, self.policy, 0.))
        self.assertFalse(path_is_clear(maps['first'], second.path, self.policy, 0.))
        self.assertEqual(self.grid.cells, floor_before)
        # Площадка внутри raised occupancy не становится допустимой на полу.
        bad_stages = dict(stages)
        bad_stages[('first', False)] = (first.path[0][:2], stages[('first', False)][1])
        self.assertEqual(optimize_route(pose, [first, second], self.grid, self.policy, 0.,
                                       surface_grids=maps, stages=bad_stages), [])

    def test_surface_overlay_is_never_opened_for_inter_surface_transit_or_home(self):
        # Первое препятствие пересекает сплошную перегородку, второе — слева.
        # Покрыть оба возможно: second -> first. Вернуться домой обычным A*
        # невозможно: повторное применение открытой поверхности запрещено.
        for row in self.grid.cells:
            row[15] = 1
        for x in range(8, 11):
            self.grid.cells[25][x] = 1
        first = obstacle('first', (2.7, 3.1), (3.5, 3.1))
        second = obstacle('second', (1.7, 5.1), (2.1, 5.1))
        maps = {name: deepcopy(self.grid) for name in ('first', 'second')}
        for y in range(13, 18):
            maps['first'].cells[y][15] = 0
        for x in range(8, 11):
            maps['second'].cells[25][x] = 0
        stages = {('first', False): ((2.3, 3.1), (3.9, 3.1)),
                  ('second', False): ((1.1, 5.1), (2.7, 5.1))}
        pose = Pose(1.1, 3.1, 0.)
        self.assertEqual(optimize_route(pose, [first, second], self.grid, self.policy, 0.,
                                       surface_grids=maps, stages=stages),
                         [('second', False), ('first', False)])
        self.assertEqual(optimize_route(pose, [first, second], self.grid, self.policy, 0.,
                                       surface_grids=maps, stages=stages, home=pose), [])

    def test_surface_map_must_match_floor_contract_and_must_not_clear_unknown(self):
        ob = obstacle('raised', (2.1, 3.1), (3.1, 3.1))
        pose = Pose(1.1, 3.1, 0.)
        for x in range(10, 16):
            self.grid.cells[15][x] = 1
        overlay = deepcopy(self.grid)
        for x in range(10, 16):
            overlay.cells[15][x] = 0
        stages = {('raised', False): ((1.5, 3.1), (3.7, 3.1))}
        invalid_maps = [replace(overlay, verified=False), replace(overlay, frame_epoch=1),
                        replace(overlay, origin=(.01, 0.)), replace(overlay, resolution=.1),
                        replace(overlay, cells=overlay.cells[:-1])]
        for invalid in invalid_maps:
            self.assertEqual(optimize_route(pose, [ob], self.grid, self.policy, 0.,
                                           surface_grids={'raised': invalid}, stages=stages), [])
        self.grid.cells[15][12] = -1
        self.assertEqual(optimize_route(pose, [ob], self.grid, self.policy, 0.,
                                       surface_grids={'raised': overlay}, stages=stages), [])


if __name__ == '__main__':
    unittest.main()
