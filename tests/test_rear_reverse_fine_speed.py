from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest


ROOT = Path(__file__).resolve().parents[1]


class RearReverseFineSpeedTests(unittest.TestCase):
    def test_reverse_pwm_floor_only_applies_while_starting(self):
        source = textwrap.dedent(
            r"""
            #include <math.h>
            #include "rear_motor/rear_reverse_pwm_floor.h"

            static int close_enough(float actual, float expected)
            {
                return fabsf(actual - expected) < 0.0001f;
            }

            int main(void)
            {
                if(!close_enough(
                        rear_reverse_apply_startup_pwm_floor(
                                -0.30f, 0.00f, -600.0f, 0.05f, 800.0f),
                        -800.0f)) return 1;
                if(!close_enough(
                        rear_reverse_apply_startup_pwm_floor(
                                -0.30f, -0.10f, -600.0f, 0.05f, 800.0f),
                        -600.0f)) return 2;
                if(!close_enough(
                        rear_reverse_apply_startup_pwm_floor(
                                -0.40f, 0.00f, -1200.0f, 0.05f, 800.0f),
                        -1200.0f)) return 3;
                if(!close_enough(
                        rear_reverse_apply_startup_pwm_floor(
                                0.30f, 0.00f, 600.0f, 0.05f, 800.0f),
                        600.0f)) return 4;
                return 0;
            }
            """
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            source_path = temp_path / "rear_reverse_pwm_floor_test.c"
            executable_path = temp_path / "rear_reverse_pwm_floor_test.exe"
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

    def test_rear_motor_uses_startup_only_reverse_floor(self):
        source = (ROOT / "code" / "rear_motor" / "rear_motor.c").read_text(
            encoding="utf-8"
        )
        header = (ROOT / "code" / "rear_motor" / "rear_motor.h").read_text(
            encoding="utf-8"
        )

        self.assertIn("#define REAR_REVERSE_STARTUP_SPEED_MPS 0.05f", header)
        self.assertIn("#define REAR_REVERSE_STARTUP_PWM_MIN 800", header)
        self.assertIn("rear_reverse_apply_startup_pwm_floor(", source)
        self.assertIn("raw_actual_mps", source)
        self.assertNotIn("REAR_REVERSE_FINE_PWM_MIN", header)


if __name__ == "__main__":
    unittest.main()
