from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class FinalRearInterfaceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.guandao = (ROOT / "code" / "guandao.c").read_text(encoding="utf-8")
        cls.main = (ROOT / "user" / "cpu0_main.c").read_text(encoding="utf-8")

    def test_odometry_integrates_pulse_yaw_pairs(self):
        start = self.guandao.index("void update_state(")
        end = self.guandao.index("void portion_1_reset", start)
        body = self.guandao[start:end]
        self.assertIn(
            "rear_motor_take_odometry_sample(&odometry_pulses, &sample_yaw)",
            body,
        )
        self.assertNotIn("rear_motor_take_odometry_pulses", body)

    def test_debug_uses_current_rear_interface_only(self):
        removed = (
            "rear_motor_get_odometry_last_sample",
            "rear_motor_get_odometry_pending_pulses",
            "rear_motor_get_odometry_rejected_samples",
            "rear_motor_get_odometry_rejected_pulses",
            "rear_motor_get_odometry_max_abs_sample",
            "rear_motor_get_total_encoder_pulses",
            "rear_motor_get_total_distance_m",
        )
        for symbol in removed:
            self.assertNotIn(symbol, self.main)
        self.assertIn("rear_motor_get_odometry_merged_samples", self.main)
        self.assertIn("rear_motor_get_odometry_total_pulses", self.main)
        self.assertIn("rear_motor_get_odometry_pending_samples", self.main)

    def test_kms_diagnostics_include_timing_pose_and_steering(self):
        self.assertIn("P3AUTO,cfg=p3save1", self.main)
        self.assertIn("pRel=%ld", self.main)
        self.assertIn("gE100=%ld", self.main)
        self.assertIn("steerAct10=%ld", self.main)
        self.assertIn("brk=%u", self.main)
        self.assertIn("brkP=%d", self.main)
        self.assertIn("brkR=%u", self.main)
        self.assertIn("serial_debug_tx_buffer[1024]", self.main)


if __name__ == "__main__":
    unittest.main()
