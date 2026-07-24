from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class HipMotorSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.header = (ROOT / "code" / "peripheral.h").read_text(encoding="utf-8", errors="replace")
        cls.peripheral = (ROOT / "code" / "peripheral.c").read_text(encoding="utf-8", errors="replace")
        cls.isr = (ROOT / "user" / "isr.c").read_text(encoding="utf-8", errors="replace")
        rear_motor_path = ROOT / "code" / "rear_motor" / "rear_motor.c"
        cls.rear_motor = rear_motor_path.read_text(encoding="utf-8", errors="replace") if rear_motor_path.exists() else None

    def test_hip_pwm_pin_mapping_matches_p5_connector(self):
        self.assertRegex(self.header, r"#define\s+PWM_L1\s+\(ATOM0_CH2_P21_4\)")
        self.assertRegex(self.header, r"#define\s+PWM_L2\s+\(ATOM0_CH3_P21_5\)")
        self.assertRegex(self.header, r"#define\s+PWM_R1\s+\(ATOM0_CH0_P21_2\)")
        self.assertRegex(self.header, r"#define\s+PWM_R2\s+\(ATOM0_CH1_P21_3\)")
        self.assertNotIn("MOTOR_GPIO_L", self.header)
        self.assertNotIn("MOTOR_GPIO_R", self.header)

    def test_motor_paths_use_four_pwm_inputs(self):
        sources = [self.peripheral]
        if self.rear_motor is not None:
            sources.append(self.rear_motor)
        for source in sources:
            for channel in ("PWM_L1", "PWM_L2", "PWM_R1", "PWM_R2"):
                self.assertIn(f"pwm_init({channel}, 17000, 0)", source)
            self.assertNotIn("gpio_set_level(MOTOR_GPIO", source)
            self.assertNotIn("gpio_init(MOTOR_GPIO", source)

    def test_stop_path_drives_all_hip_inputs_low(self):
        if self.rear_motor is None:
            self.skipTest("branch has no rear_motor module")
        stop_body = self.rear_motor.split("void rear_motor_stop(void)", 1)[1].split("}", 1)[0]
        for channel in ("PWM_L1", "PWM_L2", "PWM_R1", "PWM_R2"):
            self.assertIn(f"pwm_set_duty({channel}, 0)", stop_body)

    def test_idle_mode_clears_stale_steering_pwm(self):
        idle_body = self.isr.split("case IDLE:", 1)[1].split("break;", 1)[0]
        self.assertIn("VeerMoter_Set(0);", idle_body)


if __name__ == "__main__":
    unittest.main()
