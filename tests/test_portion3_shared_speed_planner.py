import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def function_body(source: str, start_marker: str, end_marker: str) -> str:
    start = source.index(start_marker)
    end = source.index(end_marker, start)
    return source[start:end]


class Portion3SharedSpeedPlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (ROOT / "code" / "guandao.c").read_text(encoding="utf-8")
        cls.pursuit = function_body(
            cls.source,
            "void pursuit_contral_mode(",
            "float speed_calculate(",
        )
        cls.portion1_reset = function_body(
            cls.source,
            "void portion_1_reset(void)",
            "void portion_1(void)",
        )
        cls.portion3_reset = function_body(
            cls.source,
            "void portion3_return_reset(void)",
            "void Guandao_Points_Show(",
        )

    def test_portion1_and_portion3_share_one_continuous_planner(self):
        self.assertIn(
            "static guandao_speed_planner_t portion1_speed_planner = {MIN_SPEED, 0u};",
            self.source,
        )
        self.assertNotIn(
            "guandao_speed_planner_reset(&portion1_speed_planner",
            self.portion1_reset,
        )
        self.assertNotIn(
            "guandao_speed_planner_reset(&portion1_speed_planner",
            self.portion3_reset,
        )
        self.assertIn(
            "if(route_setting_choice == 0 || route_setting_choice == 2)",
            self.pursuit,
        )

    def test_portion3_uses_portion1_turn_window_and_hysteresis(self):
        self.assertIn(
            "(route_setting_choice == 0 || route_setting_choice == 2)",
            self.pursuit,
        )
        self.assertIn("GUANDAO_P1_TURN_WINDOW_M", self.pursuit)
        self.assertIn(
            "guandao_speed_turn_level(\n                &portion1_speed_planner, upcoming_turn)",
            self.pursuit,
        )

    def test_portion3_uses_portion1_curve_ratios(self):
        self.assertNotIn("GUANDAO_KMS_CURVE_SPEED_RATIO", self.source)
        self.assertNotIn("GUANDAO_KMS_SHARP_TURN_SPEED_RATIO", self.source)
        self.assertNotIn("GUANDAO_KMS_ACCUM_TURN_", self.source)
        self.assertIn(
            "float curve_speed_ratio = GUANDAO_KMY_CURVE_SPEED_RATIO;",
            self.pursuit,
        )
        self.assertIn(
            "float sharp_turn_speed_ratio = GUANDAO_KMY_SHARP_TURN_SPEED_RATIO;",
            self.pursuit,
        )
        self.assertIn(
            "float accum_turn_slow_ratio = GUANDAO_KMY_ACCUM_TURN_SLOW_RATIO;",
            self.pursuit,
        )
        self.assertIn(
            "float accum_turn_medium_ratio = GUANDAO_KMY_ACCUM_TURN_MEDIUM_RATIO;",
            self.pursuit,
        )
        self.assertIn(
            "float accum_turn_sharp_ratio = GUANDAO_KMY_ACCUM_TURN_SHARP_RATIO;",
            self.pursuit,
        )

    def test_portion3_uses_shared_rate_limiter_without_entry_reset(self):
        self.assertIn(
            "if(route_setting_choice == 0 || route_setting_choice == 2)",
            self.pursuit,
        )
        self.assertIn(
            "guandao_speed_rate_limit(\n"
            "                   &portion1_speed_planner, v_center,",
            self.pursuit,
        )
        self.assertNotIn(
            "guandao_speed_planner_reset(&portion1_speed_planner",
            self.source,
        )


if __name__ == "__main__":
    unittest.main()
