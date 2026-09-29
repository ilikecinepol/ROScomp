"""Идентичность и свидетельства профиля; фикстуры только синтетические."""
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import tempfile
import unittest

from wolf_go2.profile import RobotProfile


def synthetic_profile():
    """Этот объект не является калибровкой или профилем физического робота."""
    keys = ('identity', 'motion_response', 'geometry_frames', 'localization', 'support_geometry')
    clock = dict(scale=1., offset=0., verified=True,
                 evidence='synthetic unit test only', clock_domain='linux_monotonic',
                 pi_boot_id='synthetic-boot-for-unit-test')
    return RobotProfile('synthetic-robot-for-unit-test',
        verified={key: True for key in keys},
        evidence={key: 'synthetic unit test only' for key in keys},
        geometry=dict(pose_child_frame='base_link', point_frame='synthetic-world',
                      pose_frame='synthetic-world', up_axis='+z', units='m'),
        time_contracts={'ROBOTODOM': dict(clock), 'ULIDAR_ARRAY': dict(clock)},
        expected_identity={'serial_number': 'SYNTHETIC-TEST-ONLY'})


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='wolf-go2-profile-test-')
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'synthetic-profile.json'

    def load(self, value, encoding='utf-8'):
        self.path.write_text(json.dumps(value), encoding=encoding)
        return RobotProfile.load(self.path)

    def test_default_profile_does_not_enable_physical_skills(self):
        profile = RobotProfile('newly-assigned-robot')
        self.assertEqual(profile.policy.validated_skills, ())
        self.assertTrue(profile.readiness(profile.robot_id))
        contract = profile.geometry_contract(profile.robot_id, .1)
        for key in ('geometry_frames_validated', 'identity_verified', 'source_time_validated'):
            self.assertFalse(contract[key])

    def test_valid_synthetic_roundtrip_bom_and_matching_session(self):
        profile = self.load(asdict(synthetic_profile()), encoding='utf-8-sig')
        self.assertEqual(profile.policy.validated_skills, ())
        self.assertEqual(profile.readiness(profile.robot_id), [])
        self.assertEqual(profile.session_errors(profile.robot_id,
            {'serial_number': 'SYNTHETIC-TEST-ONLY'}, 'synthetic-boot-for-unit-test'), [])

    def test_robot_assignment_and_serial_are_independent_checks(self):
        profile = synthetic_profile()
        for assigned, identity in ((profile.robot_id, {'serial_number': 'another-device'}),
                                   ('another-assignment', profile.expected_identity),
                                   (profile.robot_id, {})):
            self.assertTrue(profile.session_errors(assigned, identity, 'synthetic-boot-for-unit-test'))
        profile.expected_identity = {'software_version': 'same-on-four-robots'}
        self.assertTrue(any('серийного' in reason for reason in profile.readiness(profile.robot_id)))

    def test_reboot_invalidates_each_time_binding(self):
        profile = synthetic_profile()
        errors = profile.session_errors(profile.robot_id, profile.expected_identity, 'new-boot')
        self.assertTrue(any('ROBOTODOM' in reason for reason in errors))
        self.assertTrue(any('ULIDAR_ARRAY' in reason for reason in errors))

    def test_claim_without_evidence_does_not_become_ready(self):
        for name in synthetic_profile().verified:
            profile = synthetic_profile()
            del profile.evidence[name]
            self.assertTrue(profile.readiness(profile.robot_id), name)
            profile = synthetic_profile()
            profile.verified[name] = 'true'
            self.assertTrue(profile.readiness(profile.robot_id), name)

    def test_missing_stream_and_geometry_mismatch_block_readiness(self):
        profile = synthetic_profile()
        del profile.time_contracts['ULIDAR_ARRAY']
        self.assertTrue(profile.readiness(profile.robot_id))
        for key, value in (('pose_child_frame', 'unknown-child'), ('point_frame', 'another-map'),
                           ('up_axis', '-z'), ('units', 'mm')):
            profile = synthetic_profile()
            profile.geometry[key] = value
            self.assertTrue(profile.readiness(profile.robot_id), key)

    def test_time_contract_rejects_unverified_and_wrong_clock(self):
        for key, value in (('scale', 0.), ('scale', -1.), ('scale', float('nan')),
                           ('scale', True), ('offset', float('inf')), ('verified', False),
                           ('evidence', ''), ('clock_domain', 'wall-clock'), ('pi_boot_id', '')):
            profile = synthetic_profile()
            profile.time_contracts['ROBOTODOM'][key] = value
            with self.assertRaises(ValueError, msg=key):
                profile.validate()

    def test_profiles_cannot_embed_layout_or_raise_api_limits(self):
        for field in ('obstacles', 'waypoints', 'route', 'arena', 'structure_evidence'):
            value = asdict(synthetic_profile())
            value['geometry'][field] = []
            with self.assertRaises(ValueError, msg=field):
                self.load(value)
        for field, excessive in (('max_vx', .61), ('max_vy', .51), ('max_wz', 1.01)):
            value = asdict(synthetic_profile())
            value['policy'][field] = excessive
            with self.assertRaises(ValueError, msg=field):
                self.load(value)

    def test_unknown_fields_and_unknown_skills_rejected(self):
        base = asdict(synthetic_profile())
        for value in ([base], {**base, 'allow_any_robot': True},
                      {**base, 'policy': {'skip_health_checks': True}}):
            with self.assertRaises(ValueError):
                self.load(value)
        value = deepcopy(base)
        value['policy']['validated_skills'] = ['unreviewed-jump']
        with self.assertRaises(ValueError):
            self.load(value)


if __name__ == '__main__':
    unittest.main()
