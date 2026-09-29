"""Проверки управления без робота, сети и команд движения."""
import threading
import time
import unittest

from keyboard_client import DemoTransport, InputState, Outbox, key_name


def ready_state(now=10.):
    state = InputState()
    state.focused = True
    state.frame_time = now
    state.update_status({"phase": "ready", "armed": True, "token": 1, "telemetry_age_s": .01, "camera_age_s": .01}, now)
    return state


class SafetyTests(unittest.TestCase):
    def test_stopped_reason_does_not_flicker_on_heartbeat(self):
        state = ready_state()
        message = {**state.status, 'phase': 'stopped', 'armed': False,
                   'reason': 'Ошибка подготовки режима: mcf'}
        state.update_status(message, 10.)
        state.heartbeat(10.1)
        self.assertEqual(state.reason, message['reason'])

    def test_stop_transmits_specific_cause(self):
        state = ready_state()
        command = state.focus_lost()
        self.assertEqual(command['reason'], 'Окно не активно — остановлено')

    def test_no_automatic_arm(self):
        state = ready_state()
        state.keys = {"space", "w"}
        self.assertFalse(state.heartbeat(10.)["deadman"])

    def test_arm_requires_released_keys(self):
        state = ready_state()
        state.keys = {"space"}
        self.assertIsNone(state.request_arm(10.))
        self.assertFalse(state.arm_intent)

    def test_deadman_and_release(self):
        state = ready_state()
        self.assertEqual(state.request_arm(10.)["type"], "arm")
        state.keys = {"space", "w", "q"}
        command = state.heartbeat(10.1)
        self.assertTrue(command["deadman"])
        self.assertEqual(command["keys"], ["q", "w"])
        state.keys.clear()
        self.assertFalse(state.heartbeat(10.1)["deadman"])
        self.assertTrue(state.arm_intent)

    def test_freshness_disarms_and_cannot_auto_resume(self):
        for attribute in ("status_time", "frame_time"):
            with self.subTest(attribute=attribute):
                state = ready_state()
                state.request_arm(10.)
                state.keys = {"space", "w"}
                setattr(state, attribute, 8.)
                self.assertEqual(state.heartbeat(10.)["type"], "stop")
                self.assertFalse(state.arm_intent)
                self.assertFalse(state.keys)
                setattr(state, attribute, 10.)
                self.assertFalse(state.heartbeat(10.)["deadman"])

    def test_remote_sensor_ages_are_required_and_finite(self):
        for field in ("telemetry_age_s", "camera_age_s"):
            for value in (None, float("nan"), -1., 5.):
                state = ready_state()
                state.status[field] = value
                self.assertIsNone(state.request_arm(10.))

    def test_focus_loss_releases_everything(self):
        state = ready_state()
        state.request_arm(10.)
        state.keys = {"space", "w"}
        self.assertEqual(state.focus_lost()["type"], "stop")
        self.assertFalse(state.keys)
        state.focused = True
        self.assertFalse(state.heartbeat(10.)["deadman"])

    def test_arming_heartbeat_never_moves(self):
        state = ready_state()
        state.request_arm(10.)
        state.status["phase"] = "arming"
        state.keys = {"space", "w"}
        self.assertFalse(state.heartbeat(10.)["deadman"])

    def test_remote_stop_cancels_arm_intent(self):
        state = ready_state()
        state.request_arm(10.)
        state.update_status({**state.status, "phase": "ready", "armed": True}, 10.)
        state.update_status({**state.status, "phase": "stopped", "armed": False}, 10.)
        self.assertFalse(state.arm_intent)

    def test_old_stopped_status_during_arm_never_moves(self):
        state = ready_state()
        state.request_arm(10.)
        state.update_status({**state.status, "phase": "stopped", "armed": False}, 10.1)
        self.assertTrue(state.arm_intent)
        state.keys = {"space", "w"}
        self.assertFalse(state.heartbeat(10.1)["deadman"])
        state.update_status({**state.status, "phase": "ready", "armed": True}, 10.2)
        self.assertTrue(state.heartbeat(10.2)["deadman"])

    def test_no_arm_confirmation_eventually_stops(self):
        state = ready_state()
        state.status["phase"] = "waiting_arm"
        state.request_arm(10.)
        state.update_status(state.status, 10.9)
        state.frame_time = 10.9
        self.assertEqual(state.heartbeat(10.9)["type"], "stop")
        self.assertFalse(state.arm_intent)

    def test_russian_and_physical_keyboard(self):
        self.assertEqual(key_name("ц"), "w")
        self.assertEqual(key_name("Cyrillic_tse", 87), "w")
        self.assertEqual(key_name("space", 32), "space")


class QueueTests(unittest.TestCase):
    def test_heartbeat_coalesced_and_memory_bounded(self):
        outbox = Outbox()
        for token in range(5000):
            outbox.put({"type": "input", "token": token, "deadman": True}, now=10.)
        self.assertEqual(len(outbox.controls), 0)
        self.assertEqual(outbox.take(timeout=0, now=10.)["token"], 4999)
        self.assertIsNone(outbox.take(timeout=0, now=10.))

    def test_stop_removes_queued_arm_and_movement(self):
        outbox = Outbox()
        outbox.put({"type": "arm"}, now=10.)
        outbox.put({"type": "input", "deadman": True}, now=10.)
        outbox.put({"type": "stop"}, now=10.)
        self.assertEqual(outbox.take(timeout=0, now=10.)["type"], "stop")
        self.assertIsNone(outbox.take(timeout=0, now=10.))

    def test_stale_arm_and_input_turn_into_stop(self):
        for kind in ("arm", "input"):
            outbox = Outbox()
            outbox.put({"type": kind, "deadman": True}, now=10.)
            self.assertEqual(outbox.take(timeout=0, now=10.2)["type"], "stop")

    def test_control_queue_full_never_blocks(self):
        outbox = Outbox(control_limit=2)
        self.assertTrue(outbox.put({"type": "marker"}))
        self.assertTrue(outbox.put({"type": "marker"}))
        start = time.monotonic()
        self.assertFalse(outbox.put({"type": "marker"}))
        self.assertLess(time.monotonic() - start, .05)
        self.assertTrue(outbox.put({"type": "stop"}))

    def test_actual_emission_sequence_increases(self):
        outbox = Outbox()
        outbox.put({"type": "input"}, now=10.)
        outbox.put({"type": "marker"}, now=10.)
        first = outbox.take(timeout=0, now=10.)
        second = outbox.take(timeout=0, now=10.)
        self.assertEqual(first["seq"], 1)
        self.assertEqual(second["seq"], 2)

    def test_quit_cannot_be_removed_by_later_stop(self):
        outbox = Outbox()
        outbox.put({"type": "quit"})
        outbox.put({"type": "stop"})
        self.assertFalse(outbox.put({"type": "input"}))
        self.assertEqual(outbox.take(timeout=0)["type"], "quit")

    def test_close_wakes_waiter(self):
        outbox = Outbox()
        completed = threading.Event()
        worker = threading.Thread(target=lambda: (outbox.take(timeout=10.), completed.set()))
        worker.start()
        outbox.close()
        self.assertTrue(completed.wait(.5))
        worker.join()
        self.assertFalse(outbox.put({"type": "arm"}))

    def test_demo_never_needs_a_process(self):
        demo = DemoTransport()
        demo.start()
        self.assertIn("frame", demo.snapshot())
        self.assertFalse(demo.armed)
        demo.send({"type": "arm"})
        self.assertTrue(demo.armed)
        demo.send({"type": "quit"})
        self.assertFalse(demo.armed)
        self.assertTrue(demo.done.is_set())


if __name__ == "__main__":
    unittest.main(verbosity=2)
