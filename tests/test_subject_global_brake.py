import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class SubjectGlobalBrakeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.guandao = (ROOT / "code" / "guandao.c").read_text(encoding="utf-8")
        cls.main = (ROOT / "user" / "cpu0_main.c").read_text(encoding="utf-8")
        cls.isr = (ROOT / "user" / "isr.c").read_text(encoding="latin-1")

    def test_dispatcher_services_active_brake_before_control_modes(self):
        body = self.main.split("static void Guandao_Rear_Motor_Update(void)", 1)[1]
        body = body.split("static int32 Serial_Debug_Scale", 1)[0]
        active = body.index("rear_motor_brake_active()")
        update = body.index("rear_motor_brake_update()")
        remote = body.index("conrtol_mode == YAOKONG")
        self.assertLess(active, remote)
        self.assertLess(update, remote)

    def test_remote_neutral_does_not_start_active_brake(self):
        body = self.main.split("else if(conrtol_mode == YAOKONG)", 1)[1]
        body = body.split("else if(main_mode != Rack_Test_Mode)", 1)[0]
        self.assertNotIn("rear_motor_brake_start", body)

    def test_subject_one_stop_events_start_brake_once(self):
        self.assertIn("portion1_forward_brake_requested", self.guandao)
        self.assertIn("portion1_reverse_brake_requested", self.guandao)
        self.assertIn("portion1_park_brake_requested", self.guandao)
        self.assertGreaterEqual(self.guandao.count("rear_motor_brake_start();"), 5)

    def test_subject_one_preserves_original_reverse_entry_steering(self):
        portion_one = self.guandao.split("void portion_1(void)", 1)[1]
        portion_one = portion_one.split("void recode_waypoint(", 1)[0]
        reverse_entry = portion_one.split(
            "if(reverse_ready || final_stop_ready)", 1
        )[1]
        reverse_entry = reverse_entry.split("follow_points_show(&INS);", 1)[0]

        self.assertNotIn("portion1_reverse_entry_state", self.guandao)
        self.assertIn(
            "portion1_reverse_steer_cmd = out_servo * GUANDAO_REVERSE_STEERING_GAIN;",
            reverse_entry,
        )

    def test_subject_one_keeps_original_steering_while_braking(self):
        portion_one = self.guandao.split("void portion_1(void)", 1)[1]
        wait_state = portion_one.split(
            "if(portion1_reverse_state == 1)", 1
        )[1].split("else if(portion1_reverse_state == 2)", 1)[0]

        self.assertIn("out_servo = portion1_reverse_steer_cmd;", wait_state)

    def test_taught_reverse_blends_runtime_start_into_fixed_taught_target(self):
        prepare = self.guandao.split(
            "static void guandao_taught_reverse_prepare(void)", 1
        )[1].split("static uint8 guandao_taught_reverse_update(void)", 1)[0]

        self.assertIn("portion1_reverse_start_state.theta", prepare)
        self.assertIn("portion1_reverse_start_state.x", prepare)
        self.assertIn("portion1_reverse_start_state.y", prepare)
        self.assertIn("parking_se2_blend_pose_to_fixed(", prepare)
        self.assertIn(
            "portion1_taught_reverse_target = daoche_target_state;",
            prepare,
        )
        self.assertNotIn("&portion1_taught_reverse_target.x", prepare)

    def test_subject_one_disables_generated_reverse_plan_fallback(self):
        portion_one = self.guandao.split("void portion_1(void)", 1)[1]
        portion_one = portion_one.split("void recode_waypoint(", 1)[0]

        self.assertNotIn("auto_park_build_plan(", self.guandao)
        self.assertNotIn("guandao_reverse_prepare_plan", self.guandao)
        self.assertNotIn("guandao_reverse_execute_plan", self.guandao)
        self.assertNotIn("portion1_reverse_plan.routes[0].is_forward", portion_one)

    def test_subject_one_uses_reverse_only_target_fallback(self):
        portion_one = self.guandao.split("void portion_1(void)", 1)[1]
        wait_state = portion_one.split(
            "if(portion1_reverse_state == 1)", 1
        )[1].split("else if(portion1_reverse_state == 2)", 1)[0]
        reverse_state = portion_one.split(
            "else if(portion1_reverse_state == 2)", 1
        )[1].split("else if(portion1_reverse_state == 3)", 1)[0]

        self.assertNotIn("guandao_reverse_prepare_plan();", wait_state)
        self.assertIn("daoche_flag = 1;", wait_state)
        self.assertIn("conrtol_mode = DAOCHE;", wait_state)
        self.assertIn(
            "if(!portion1_taught_reverse_ready && daoche_target_flag)",
            reverse_state,
        )
        self.assertNotIn("guandao_reverse_execute_plan", reverse_state)

    def test_reverse_plan_diagnostic_reports_disabled(self):
        diagnostic = self.guandao.split(
            "uint8 guandao_reverse_debug_plan_ready(void)", 1
        )[1].split("uint8 guandao_reverse_debug_fail_reason(void)", 1)[0]
        self.assertIn("return 0u;", diagnostic)

    def test_subject_one_uses_original_approach_and_gate_rules(self):
        for token in (
            "#define GUANDAO_PARK_APPROACH_DIST     2.00f",
            "#define GUANDAO_PARK_APPROACH_RADIUS   1.80f",
            "#define GUANDAO_PARK_APPROACH_SPEED_FAST 10.0f",
            "#define GUANDAO_PARK_APPROACH_SPEED_MID  8.0f",
            "#define GUANDAO_PARK_APPROACH_SPEED_SLOW 5.0f",
        ):
            self.assertIn(token, self.guandao)

        self.assertNotIn("GUANDAO_PARK_GATE_LONG_LIMIT", self.guandao)
        self.assertNotIn("GUANDAO_PARK_GATE_PRESTOP_DIST", self.guandao)

    def test_route_save_braking_order_matches_route_type(self):
        recode = self.guandao.split("void guandao_recode(guandao_state * state)", 1)[1]
        recode = recode.split("void guandao_record_session_reset(void)", 1)[0]
        save_calls = list(
            re.finditer(r"Flash_Store_Mode\(route_setting_choice\);", recode)
        )
        self.assertEqual(len(save_calls), 3)

        pending_block = recode.split("if(portion3_save_pending)", 1)[1]
        pending_block = pending_block.split("// KEY4", 1)[0]
        self.assertLess(
            pending_block.index("if(rear_motor_brake_active()) return;"),
            pending_block.index("Flash_Store_Mode(route_setting_choice);"),
        )

        for save_call in save_calls[1:]:
            brake_position = recode.index(
                "rear_motor_brake_start();", save_call.end()
            )
            self.assertLess(brake_position - save_call.end(), 400)

    def test_generic_route_endpoint_has_one_shot_brake_latch(self):
        self.assertIn("guandao_trace_brake_requested", self.guandao)
        trace = self.guandao.split("void guandao_trace(guandao_state * state)", 1)[1]
        trace = trace.split("float speed_calculate", 1)[0]
        self.assertIn("guandao_debug_stop_reason == 1", trace)
        self.assertIn("guandao_debug_stop_reason == 4", trace)
        self.assertIn("rear_motor_brake_start();", trace)

    def test_emergency_key_keeps_immediate_stop(self):
        block = self.isr.split("if(Main_Key_Flag == 0)", 1)[1].split("}", 1)[0]
        self.assertIn("rear_motor_stop();", block)
        self.assertNotIn("rear_motor_brake_start", block)


if __name__ == "__main__":
    unittest.main()
