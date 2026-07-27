from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


def function_body(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


class Portion3SaveConsistencyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.guandao_c = (ROOT / "code" / "guandao.c").read_text(encoding="utf-8")
        cls.display_c = (ROOT / "code" / "display.c").read_text(encoding="utf-8")
        cls.flash_c = (ROOT / "code" / "flash.c").read_text(encoding="utf-8")
        cls.main_c = (ROOT / "user" / "cpu0_main.c").read_text(encoding="utf-8")

    def test_save_locks_recording_until_new_session(self):
        recode_body = function_body(
            self.guandao_c,
            "void guandao_recode(guandao_state * state)",
            "void guandao_record_session_reset(void)",
        )
        reset_body = function_body(
            self.guandao_c,
            "void guandao_record_session_reset(void)",
            "uint8 portion2_points_build(void)",
        )

        self.assertIn("static uint8 guandao_record_saved = 0;", self.guandao_c)
        self.assertEqual(recode_body.count("guandao_record_saved = 1;"), 2)
        self.assertLess(
            recode_body.index("if(guandao_record_saved) return;"),
            recode_body.index("update_state(p  , &guandao_ecd);"),
        )
        sbus_start = recode_body.index("if(x6f_out[2] == 200)")
        sbus_end = recode_body.index("\n    else", sbus_start)
        sbus_body = recode_body[sbus_start:sbus_end]
        self.assertLess(
            sbus_body.index("guandao_record_saved = 1;"),
            sbus_body.index("recode_waypoint(p);"),
        )
        self.assertIn("if(guandao_record_saved) return;", sbus_body)
        self.assertIn("guandao_record_saved = 0;", reset_body)

    def test_record_menu_starts_a_fresh_session(self):
        menu_body = function_body(
            self.display_c,
            "void Menu_Recode_Points(void)",
            "void Menu_1(void)",
        )

        self.assertEqual(menu_body.count("guandao_record_session_reset();"), 1)
        self.assertIn(
            "        }\n"
            "        guandao_record_session_reset();\n"
            "        angle_control_select_route(route_setting_choice);",
            menu_body,
        )

    def test_flash_reload_preserves_exact_saved_length(self):
        read_body = function_body(
            self.flash_c,
            "void Flash_Read_portion_3points(void)",
            "void Flash_Read_INSpoints(void)",
        )

        self.assertIn(
            "portion_3.length_index = flash_clamp_route_length(stored_length);",
            read_body,
        )
        self.assertNotIn("stored_length - 1", read_body)
        self.assertIn(
            "(int16)(flash_union_buffer[1].uint32_type & 0xFFFFu) != continuation_count",
            read_body,
        )

    def test_portion3_serial_marker_identifies_fix(self):
        self.assertIn("P3AUTO,cfg=p3save2", self.main_c)


if __name__ == "__main__":
    unittest.main()
