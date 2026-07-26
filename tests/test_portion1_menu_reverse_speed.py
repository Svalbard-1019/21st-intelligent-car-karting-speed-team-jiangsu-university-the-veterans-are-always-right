from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest


ROOT = Path(__file__).resolve().parents[1]


class Portion1MenuReverseSpeedTests(unittest.TestCase):
    def test_menu_speed_builds_safe_cruise_and_fine_targets(self):
        source = textwrap.dedent(
            r"""
            #include <math.h>
            #include "guandao_reverse_speed_planner.h"

            static int close_enough(float actual, float expected)
            {
                return fabsf(actual - expected) < 0.0001f;
            }

            int main(void)
            {
                guandao_reverse_speed_plan_t plan;

                plan = guandao_reverse_speed_plan(-40.0f);
                if(!close_enough(plan.cruise_units, -40.0f)) return 1;
                if(!close_enough(plan.fine_units, -3.0f)) return 2;

                plan = guandao_reverse_speed_plan(-4.0f);
                if(!close_enough(plan.cruise_units, -4.0f)) return 3;
                if(!close_enough(plan.fine_units, -3.0f)) return 4;

                plan = guandao_reverse_speed_plan(-2.0f);
                if(!close_enough(plan.cruise_units, -2.0f)) return 5;
                if(!close_enough(plan.fine_units, -1.5f)) return 6;

                plan = guandao_reverse_speed_plan(-50.0f);
                if(!close_enough(plan.cruise_units, -40.0f)) return 7;
                plan = guandao_reverse_speed_plan(0.0f);
                if(!close_enough(plan.cruise_units, -2.0f)) return 8;
                return 0;
            }
            """
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            source_path = temp_path / "portion1_menu_reverse_speed_test.c"
            executable_path = temp_path / "portion1_menu_reverse_speed_test.exe"
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

    def test_all_automatic_reverse_paths_use_menu_plan(self):
        source = (ROOT / "code" / "guandao.c").read_text(encoding="utf-8")

        self.assertIn("guandao_current_reverse_speed_plan()", source)
        self.assertIn("reverse_plan.cruise_units", source)
        self.assertIn("reverse_plan.fine_units", source)
        self.assertNotIn("GUANDAO_TAUGHT_REVERSE_SPEED", source)
        self.assertNotIn("GUANDAO_TAUGHT_REVERSE_FINE_SPEED", source)
        self.assertNotIn("GUANDAO_REVERSE_SPEED_UNITS", source)

    def test_menu_and_flash_enforce_reverse_speed_range(self):
        display = (ROOT / "code" / "display.c").read_text(encoding="utf-8")
        flash = (ROOT / "code" / "flash.c").read_text(encoding="utf-8")

        self.assertIn("if(control[1] > -2) control[1] = -2;", display)
        self.assertIn("if(control[1] < -40) control[1] = -40;", display)
        self.assertIn("#define FLASH_REVERSE_SPEED_DEFAULT     (-4)", flash)
        self.assertIn("#define FLASH_REVERSE_SPEED_MIN         (-40)", flash)
        self.assertIn("#define FLASH_REVERSE_SPEED_MAX         (-2)", flash)
        self.assertIn("control[1] = FLASH_REVERSE_SPEED_DEFAULT;", flash)


if __name__ == "__main__":
    unittest.main()
