import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class RearGlobalBrakeSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (ROOT / "code" / "rear_motor" / "rear_motor.c").read_text(
            encoding="utf-8"
        )
        cls.header = (ROOT / "code" / "rear_motor" / "rear_motor.h").read_text(
            encoding="utf-8"
        )

    def test_public_nonblocking_brake_api_is_exposed(self):
        for declaration in (
            "void rear_motor_brake_start(void);",
            "void rear_motor_brake_update(void);",
            "uint8 rear_motor_brake_active(void);",
            "uint8 rear_motor_brake_reason(void);",
            "uint32 rear_motor_brake_elapsed_ms(void);",
            "int16 rear_motor_brake_pwm(void);",
            "float rear_motor_brake_end_raw_mps(void);",
        ):
            self.assertIn(declaration, self.header)

    def test_brake_uses_verified_limits_and_three_pwm_levels(self):
        expected_defines = {
            "REAR_BRAKE_TIMEOUT_MS": "700u",
            "REAR_BRAKE_HIGH_REVERSE_GUARD_MS": "600u",
            "REAR_BRAKE_HIGH_SPEED_MPS": "3.75f",
            "REAR_BRAKE_STOP_SPEED_MPS": "0.05f",
            "REAR_BRAKE_REVERSE_MPS": "0.05f",
            "REAR_BRAKE_PWM_HIGH": "2500",
            "REAR_BRAKE_PWM_MID": "1800",
            "REAR_BRAKE_PWM_LOW": "1000",
        }
        for name, value in expected_defines.items():
            self.assertRegex(
                self.header,
                rf"(?m)^#define\s+{name}\s+{re.escape(value)}$",
            )

    def test_high_speed_reverse_exit_is_guarded_and_timeout_is_final(self):
        self.assertIn(
            "brake_start_speed_mps >= REAR_BRAKE_HIGH_SPEED_MPS",
            self.source,
        )
        self.assertIn(
            "brake_elapsed_ms >= REAR_BRAKE_HIGH_REVERSE_GUARD_MS",
            self.source,
        )
        self.assertIn(
            "brake_elapsed_ms >= REAR_BRAKE_TIMEOUT_MS",
            self.source,
        )
        self.assertIn("REAR_BRAKE_REASON_TIMEOUT", self.source)
        self.assertIn("REAR_BRAKE_REASON_REVERSE", self.source)
        self.assertIn("REAR_BRAKE_REASON_LOW_SPEED", self.source)

    def test_brake_start_does_not_reverse_an_already_stationary_car(self):
        start = self.source.index("void rear_motor_brake_start(void)")
        end = self.source.index("void rear_motor_brake_update(void)", start)
        body = self.source[start:end]
        self.assertIn(
            "brake_start_speed_mps <= REAR_BRAKE_STOP_SPEED_MPS",
            body,
        )
        self.assertIn(
            "fabsf(raw_actual_mps) <= REAR_BRAKE_STOP_SPEED_MPS",
            body,
        )
        self.assertIn(
            "rear_motor_brake_finish(REAR_BRAKE_REASON_LOW_SPEED, raw_actual_mps);",
            body,
        )

    def test_brake_finishes_through_four_channel_stop(self):
        finish_start = self.source.index("static void rear_motor_brake_finish(")
        finish_end = self.source.index("void rear_motor_brake_start(void)", finish_start)
        finish_body = self.source[finish_start:finish_end]
        self.assertIn("rear_motor_stop();", finish_body)
        self.assertIn("brake_end_raw_mps = raw_speed_mps;", finish_body)

        stop_start = self.source.index("void rear_motor_stop(void)")
        stop_end = self.source.index("void rear_motor_set_target_mps", stop_start)
        stop_body = self.source[stop_start:stop_end]
        for channel in ("PWM_L1", "PWM_L2", "PWM_R1", "PWM_R2"):
            self.assertIn(f"pwm_set_duty({channel}, 0);", stop_body)
        self.assertIn("brake_active = 0;", stop_body)


if __name__ == "__main__":
    unittest.main()
