from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest


ROOT = Path(__file__).resolve().parents[1]


class Portion1SpeedPlannerTests(unittest.TestCase):
    def test_hysteresis_and_asymmetric_rate_limit(self):
        source = textwrap.dedent(
            r"""
            #include <math.h>
            #include "guandao_speed_planner.h"

            static int close_enough(float actual, float expected)
            {
                return fabsf(actual - expected) < 0.0001f;
            }

            int main(void)
            {
                guandao_speed_planner_t planner;
                guandao_speed_planner_reset(&planner, 35.0f);

                if(guandao_speed_turn_level(&planner, 46.0f) != 2u) return 1;
                if(guandao_speed_turn_level(&planner, 42.0f) != 2u) return 2;
                if(guandao_speed_turn_level(&planner, 34.0f) != 1u) return 3;
                if(guandao_speed_turn_level(&planner, 18.0f) != 1u) return 4;
                if(guandao_speed_turn_level(&planner, 14.0f) != 0u) return 5;

                planner.command = 20.0f;
                if(!close_enough(
                        guandao_speed_rate_limit(&planner, 35.0f, 0.5f, 12.0f, 40.0f),
                        26.0f)) return 6;
                if(!close_enough(
                        guandao_speed_rate_limit(&planner, 10.0f, 0.25f, 12.0f, 40.0f),
                        16.0f)) return 7;
                return 0;
            }
            """
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            source_path = temp_path / "portion1_speed_planner_test.c"
            executable_path = temp_path / "portion1_speed_planner_test.exe"
            source_path.write_text(source, encoding="utf-8")

            compile_result = subprocess.run(
                [
                    "gcc",
                    "-std=c99",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-I",
                    str(ROOT / "code"),
                    str(source_path),
                    "-o",
                    str(executable_path),
                ],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                check=False,
            )
            self.assertEqual(
                compile_result.returncode,
                0,
                msg=compile_result.stdout + compile_result.stderr,
            )

            run_result = subprocess.run(
                [str(executable_path)],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                check=False,
            )
            self.assertEqual(
                run_result.returncode,
                0,
                msg=run_result.stdout + run_result.stderr,
            )

    def test_portion1_and_portion3_share_distance_window_and_speed_ratios(self):
        source = (ROOT / "code" / "guandao.c").read_text(encoding="utf-8")

        self.assertIn("GUANDAO_P1_TURN_WINDOW_M", source)
        self.assertIn("GUANDAO_P1_TURN_DEADBAND_DEG", source)
        self.assertIn("#define GUANDAO_P1_ACCEL_UNITS_PER_S    35.0f", source)
        self.assertIn("#define GUANDAO_PARK_APPROACH_DIST     2.00f", source)
        self.assertIn("guandao_portion1_distance_turn(", source)
        self.assertIn("fabsf(upcoming_turn) < GUANDAO_ACCUM_TURN_SLOW_ANGLE", source)
        self.assertIn("guandao_speed_turn_level(", source)
        self.assertIn("guandao_speed_rate_limit(", source)
        self.assertIn("GUANDAO_KMY_CURVE_SPEED_RATIO      0.80f", source)
        self.assertIn("GUANDAO_KMY_SHARP_TURN_SPEED_RATIO 0.65f", source)
        self.assertIn("GUANDAO_KMY_ACCUM_TURN_SLOW_RATIO  0.85f", source)
        self.assertIn("GUANDAO_KMY_ACCUM_TURN_MEDIUM_RATIO 0.75f", source)
        self.assertIn("GUANDAO_KMY_ACCUM_TURN_SHARP_RATIO 0.65f", source)
        self.assertIn("GUANDAO_HAIRPIN_SPEED_RATIO    0.60f", source)
        self.assertNotIn("GUANDAO_KMS_ACCUM_TURN_", source)
        self.assertIn(
            "(route_setting_choice == 0 || route_setting_choice == 2)",
            source,
        )
        self.assertNotIn("base_speed * 0.75f", source)
        self.assertGreaterEqual(source.count("base_speed * curve_speed_ratio"), 2)

    def test_portion1_serial_line_exposes_speed_and_reverse_diagnostics(self):
        source = (ROOT / "user" / "cpu0_main.c").read_text(encoding="utf-8")
        auto_start = source.index('"AUTO,')
        auto_end = source.index("\\r\\n", auto_start)
        auto_format = source[auto_start:auto_end]

        for field in (
            "cfg=p1spd8",
            "pwm=%d",
            "pwmReq=%d",
            "turn10=%ld",
            "turnLv=%u",
            "req100=%ld",
            "cmd100=%ld",
            "revSt=%u",
            "revPlan=%u",
            "revIdx=%d",
            "revLen=%d",
            "revD100=%ld",
            "revYaw10=%ld",
            "revCmd10=%ld",
            "revLd100=%ld",
        ):
            self.assertIn(field, auto_format)
        self.assertIn("guandao_reverse_debug_steer_command()", source)
        self.assertIn("rear_motor_get_requested_pwm()", source)


if __name__ == "__main__":
    unittest.main()
