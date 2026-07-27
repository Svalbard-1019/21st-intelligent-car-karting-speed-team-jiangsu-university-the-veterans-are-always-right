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

    def test_route_save_starts_brake_after_flash_write(self):
        save_calls = list(
            re.finditer(r"Flash_Store_Mode\(route_setting_choice\);", self.guandao)
        )
        self.assertEqual(len(save_calls), 2)
        for save_call in save_calls:
            brake_position = self.guandao.index(
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
