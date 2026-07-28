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

    def test_portion3_uses_lower_curve_ratios_without_losing_shared_planning(self):
        self.assertRegex(
            self.source,
            r"#define\s+GUANDAO_P3_CURVE_SPEED_RATIO\s+0\.72f",
        )
        self.assertRegex(
            self.source,
            r"#define\s+GUANDAO_P3_SHARP_TURN_SPEED_RATIO\s+0\.57f",
        )
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
        self.assertIn(
            "curve_speed_ratio = GUANDAO_P3_CURVE_SPEED_RATIO;",
            self.pursuit,
        )
        self.assertIn(
            "sharp_turn_speed_ratio = GUANDAO_P3_SHARP_TURN_SPEED_RATIO;",
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

    def test_portion3_stops_after_crossing_the_final_segment_gate(self):
        terminal_pass = function_body(
            self.source,
            "static uint8 guandao_portion3_terminal_passed(",
            "static AutoParkPose guandao_pose_from_state(",
        )
        self.assertRegex(
            self.source,
            r"#define\s+PORTION3_FINAL_CROSS_TRACK\s+0\.75f",
        )
        self.assertIn(
            "if(target_index <= 0 || target_index >= route_length) return 0;",
            terminal_pass,
        )
        self.assertNotIn(
            "if(target_index <= 0 || target_index >= route_length - 1) return 0;",
            terminal_pass,
        )
        self.assertIn(
            "cross_track_limit = target_index >= route_length - 1",
            terminal_pass,
        )
        self.assertIn(
            "if(target_index < route_length - 1",
            terminal_pass,
        )
        self.assertIn(
            "if(state->current_point_index >= route_length)",
            self.pursuit,
        )
        final_cross_stop = self.pursuit.index(
            "if(state->current_point_index >= route_length)"
        )
        terminal_target_reload = self.pursuit.index(
            "target_point = guandao_route_point(state, state->current_point_index);",
            self.pursuit.index("guandao_portion3_terminal_passed("),
        )
        self.assertLess(final_cross_stop, terminal_target_reload)

    def test_portion3_keeps_the_configured_final_slowdown_distance(self):
        self.assertNotIn("PORTION3_FINAL_SLOW_DIST", self.source)
        self.assertNotIn("final_slow_dist", self.pursuit)
        self.assertIn(
            "if (dist_to_final < final_dsts",
            self.pursuit,
        )
        self.assertIn(
            "base_speed * (dist_to_final / final_dsts)",
            self.pursuit,
        )


if __name__ == "__main__":
    unittest.main()
