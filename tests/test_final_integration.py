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

    def test_subject_three_has_800_point_storage(self):
        self.assertIn("#define MAX_LENGTH_INDEX                       800", self.guandao_h)
        self.assertIn("FLASH_ROUTE_FIRST_PAGE_POINTS    (500)", self.flash_c)
        self.assertIn("RECODE_PORTION_THREE_CONTINUATION", self.flash_h)

    def test_subject_modes_keep_independent_record_spacing(self):
        self.assertIn("PORTION3_RECORD_THRESHOLD      0.20f", self.guandao_c)
        self.assertIn("float recode_threshold = 0.3f", self.guandao_c)
        self.assertIn("state == &portion_3", self.guandao_c)

    def test_subject_three_uses_distance_preview_and_stable_return_heading(self):
        self.assertIn("guandao_preview_steps_for_distance", self.guandao_c)
        self.assertIn("PORTION3_HEADING_BASELINE      0.80f", self.guandao_c)
        self.assertIn("heading_baseline >= PORTION3_HEADING_BASELINE", self.guandao_c)

    def test_kmy_parking_modules_are_preserved(self):
        self.assertIn('#include "auto_park_plan.h"', self.guandao_c)
        self.assertIn('#include "parking_se2.h"', self.guandao_c)
        self.assertIn("portion1_taught_reverse_map", self.guandao_c)

    def test_serial_debug_is_nonblocking_and_uses_fixed_module(self):
        self.assertNotIn("uart_write_string(DEBUG_UART_INDEX, line)", self.main_c)
        self.assertIn("IfxAsclin_getAddress", self.main_c)
        self.assertIn("Serial_Debug_Service();", self.main_c)


if __name__ == "__main__":
    unittest.main()
