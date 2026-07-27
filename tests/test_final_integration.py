from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class FinalIntegrationSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.guandao_h = (ROOT / "code" / "guandao.h").read_text(encoding="utf-8")
        cls.guandao_c = (ROOT / "code" / "guandao.c").read_text(encoding="utf-8")
        cls.flash_h = (ROOT / "code" / "flash.h").read_text(encoding="utf-8")
        cls.flash_c = (ROOT / "code" / "flash.c").read_text(encoding="utf-8")
        cls.main_c = (ROOT / "user" / "cpu0_main.c").read_text(encoding="utf-8")
        cls.display_c = (ROOT / "code" / "display.c").read_text(encoding="utf-8")
        cls.remote_c = (ROOT / "code" / "RemteControl.c").read_text(encoding="utf-8")
        cls.angle_h = (ROOT / "code" / "angle_control.h").read_text(encoding="utf-8")
        cls.angle_c = (ROOT / "code" / "angle_control.c").read_text(encoding="utf-8")
        cls.rear_h = (ROOT / "code" / "rear_motor" / "rear_motor.h").read_text(encoding="utf-8")
        cls.rear_c = (ROOT / "code" / "rear_motor" / "rear_motor.c").read_text(encoding="utf-8")
        pursuit_start = cls.guandao_c.index("void pursuit_contral_mode(")
        pursuit_end = cls.guandao_c.index("float speed_calculate(", pursuit_start)
        cls.pursuit = cls.guandao_c[pursuit_start:pursuit_end]

    def test_subject_three_has_800_point_storage(self):
        self.assertIn("#define MAX_LENGTH_INDEX                       800", self.guandao_h)
        self.assertIn("FLASH_ROUTE_FIRST_PAGE_POINTS    (500)", self.flash_c)
        self.assertIn("RECODE_PORTION_THREE_CONTINUATION", self.flash_h)

    def test_subject_modes_keep_independent_record_spacing(self):
        self.assertIn("PORTION3_RECORD_THRESHOLD      0.20f", self.guandao_c)
        self.assertIn("float recode_threshold = 0.3f", self.guandao_c)
        self.assertIn("state == &portion_3", self.guandao_c)

    def test_subject_three_uses_latest_return_frame_and_preview(self):
        self.assertIn("PORTION3_PURSUIT_THRESHOLD     0.25f", self.guandao_c)
        self.assertIn("PORTION3_FAST_PREVIEW_STEPS    7", self.guandao_c)
        self.assertIn("PORTION3_SHARP_PREVIEW_STEPS   5", self.guandao_c)
        self.assertIn(
            "return_heading = guandao_normalize_angle(origin.theta + 180.0f);",
            self.guandao_c,
        )
        self.assertNotIn("guandao_preview_steps_for_distance", self.guandao_c)
        self.assertNotIn("heading_baseline", self.guandao_c)

    def test_subject_three_has_latest_speed_and_finish_rules(self):
        self.assertNotIn("PORTION3_CURVE_SPEED_FLOOR_RATIO", self.guandao_c)
        self.assertIn("GUANDAO_KMY_CURVE_SPEED_RATIO      0.80f", self.guandao_c)
        self.assertIn("GUANDAO_KMS_CURVE_SPEED_RATIO      0.70f", self.guandao_c)
        self.assertIn("GUANDAO_KMY_SHARP_TURN_SPEED_RATIO 0.65f", self.guandao_c)
        self.assertIn("GUANDAO_KMS_SHARP_TURN_SPEED_RATIO 0.55f", self.guandao_c)
        self.assertIn("route_setting_choice == 2", self.guandao_c)
        self.assertIn("PORTION3_RETURN_TRIM_DIST      0.5f", self.guandao_c)
        self.assertIn("uint8 hold_portion3_final_point", self.guandao_c)
        self.assertIn("dist_to_final <= PORTION3_FINAL_STOP_DIST", self.guandao_c)
        self.assertIn(
            "route_setting_choice == 2 && guandao_debug_stop_reason == 8",
            self.guandao_c,
        )

    def test_subject_three_has_terminal_pass_guard(self):
        self.assertIn("PORTION3_TERMINAL_PASS_POINTS  3", self.guandao_c)
        self.assertIn("guandao_portion3_terminal_passed", self.guandao_c)
        self.assertIn("terminal_pass_advanced", self.guandao_c)
        self.assertIn("guandao_debug_stop_reason = 9", self.guandao_c)

    def test_rear_speed_profiles_are_isolated_by_subject(self):
        for token in (
            "REAR_KMY_KP                 4.0f",
            "REAR_KMY_KI                 0.6f",
            "REAR_KMY_KD                 0.12f",
            "REAR_KMY_FF_GAIN            8.0f",
            "REAR_KMY_PWM_RATE_LIMIT     600",
            "REAR_KMS_KP                 10.0f",
            "REAR_KMS_KI                 0.3f",
            "REAR_KMS_KD                 0.8f",
            "REAR_KMS_FF_GAIN            13.0f",
            "REAR_KMS_PWM_RATE_LIMIT     1000",
        ):
            self.assertIn(token, self.rear_h)
        self.assertIn("rear_motor_select_route(uint8 route_choice)", self.rear_c)
        self.assertIn("rear_motor_select_route((conrtol_mode == GUANDAO", self.main_c)

    def test_center_odometry_is_limited_to_subject_three(self):
        self.assertIn('#include "rear_motor/rear_left_wheel_odometry.h"', self.guandao_c)
        self.assertIn("state == &portion_3", self.guandao_c)
        self.assertIn("rear_left_wheel_to_center_distance", self.guandao_c)
        self.assertIn("rear_left_wheel_odometry_reset", self.guandao_c)

    def test_tracking_loop_keeps_kms_smoothing_guards(self):
        self.assertNotIn("ips200_show_float", self.pursuit)
        self.assertIn(
            "guandao_elapsed_ms(steer_now_ms, last_steer_limit_ms)",
            self.pursuit,
        )
        self.assertIn("if(steer_elapsed_ms >= 1u)", self.pursuit)
        self.assertNotIn("guandao_update_portion3_arc_state", self.pursuit)
        self.assertNotIn("portion3_hold_arc_steering", self.pursuit)
        show_start = self.guandao_c.index("void follow_points_show(")
        show_end = self.guandao_c.index("void Key_Recode_Point(", show_start)
        show_body = self.guandao_c[show_start:show_end]
        self.assertIn("elapsed_ms < 200u", show_body)
        self.assertIn("display_index = p->current_point_index", show_body)
        self.assertNotIn("INS.current_point_index", show_body)

    def test_common_rear_driver_keeps_differential_feedforward(self):
        self.assertIn("REAR_DIFF_PWM_GAIN", self.rear_h)
        self.assertIn("float diff_val = out_v_l - out_v_r", self.rear_c)
        self.assertIn("pwm_l = current_pwm + diff_pwm", self.rear_c)
        self.assertIn("pwm_r = current_pwm - diff_pwm", self.rear_c)

    def test_kmy_parking_modules_are_preserved(self):
        self.assertIn('#include "auto_park_plan.h"', self.guandao_c)
        self.assertIn('#include "parking_se2.h"', self.guandao_c)
        self.assertIn("portion1_taught_reverse_map", self.guandao_c)

    def test_serial_debug_is_nonblocking_and_uses_fixed_module(self):
        self.assertNotIn("uart_write_string(DEBUG_UART_INDEX, line)", self.main_c)
        self.assertIn("IfxAsclin_getAddress", self.main_c)
        self.assertIn("Serial_Debug_Service();", self.main_c)
        self.assertIn("base10=%ld", self.main_c)

    def test_combined_remote_preserves_both_subject_limits(self):
        self.assertIn("sbus_rc_capture_neutral", self.remote_c)
        self.assertIn("(route_setting_choice == 2) ? 45.0f : 40.0f", self.remote_c)

    def test_control_values_persist_without_start_flash_rewrite(self):
        control_start = self.display_c.index("void Menu_Control_P(void)")
        control_end = self.display_c.index("void Menu_2Value(void)", control_start)
        control_body = self.display_c[control_start:control_end]
        self.assertIn("static uint8 params_dirty = 0;", control_body)
        self.assertIn("Flash_Write_pid();", control_body)
        main_start = self.main_c.index("int core0_main(void)")
        self.assertNotIn("Flash_Write_pid();", self.main_c[main_start:])

    def test_routes_keep_separate_flash_entry_points(self):
        self.assertIn("Flash_Write_INSpoints", self.flash_c)
        self.assertIn("Flash_Write_portion_3points", self.flash_c)
        self.assertIn("case 0", self.flash_c)
        self.assertIn("case 2", self.flash_c)

    def test_routes_keep_separate_flash_pages(self):
        self.assertIn("#define RECODE_MAP_POINTS_INDEX   (10)", self.flash_h)
        self.assertIn("#define RECODE_MAP_POINTS_CONTINUATION        (2)", self.flash_h)
        self.assertIn("#define RECODE_PORTION_THREE                  (4)", self.flash_h)
        self.assertIn("#define RECODE_PORTION_THREE_CONTINUATION    (5)", self.flash_h)

    def test_each_subject_selects_its_tested_steering_profile(self):
        self.assertIn("#define ANGLE_KMY_KP        500.0f", self.angle_h)
        self.assertIn("#define ANGLE_KMY_KI        18.0f", self.angle_h)
        self.assertIn("#define ANGLE_KMY_KD        27.0f", self.angle_h)
        self.assertIn("#define ANGLE_KMS_KP        1000.0f", self.angle_h)
        self.assertIn("#define ANGLE_KMS_KI        15.0f", self.angle_h)
        self.assertIn("#define ANGLE_KMS_KD        40.0f", self.angle_h)
        self.assertIn("if(route_choice == 2u)", self.angle_c)
        self.assertIn("angle_ff_gain = ANGLE_KMS_FF_GAIN", self.angle_c)
        self.assertGreaterEqual(
            self.display_c.count("angle_control_select_route(route_setting_choice)"),
            3,
        )


if __name__ == "__main__":
    unittest.main()
