from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest


ROOT = Path(__file__).resolve().parents[1]


class RearPwmSlewTests(unittest.TestCase):
    def test_drive_pwm_accelerates_and_releases_with_separate_limits(self):
        source = textwrap.dedent(
            r"""
            #include "rear_motor/rear_pwm_slew.h"

            int main(void)
            {
                if(rear_pwm_slew_step(1000, 2500, 600, 300) != 1600) return 1;
                if(rear_pwm_slew_step(2500, 1000, 600, 300) != 2200) return 2;
                if(rear_pwm_slew_step(-2500, -1000, 600, 300) != -2200) return 3;
                if(rear_pwm_slew_step(0, -2500, 600, 300) != -600) return 4;
                if(rear_pwm_slew_step(2500, 0, 1000, 1000) != 1500) return 5;
                if(rear_pwm_limit_request_sign(1.0f, -500) != 0) return 6;
                if(rear_pwm_limit_request_sign(-1.0f, 500) != 0) return 7;
                if(rear_pwm_limit_request_sign(1.0f, 500) != 500) return 8;
                if(rear_pwm_limit_request_sign(-1.0f, -500) != -500) return 9;
                return 0;
            }
            """
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            source_path = temp_path / "rear_pwm_slew_test.c"
            executable_path = temp_path / "rear_pwm_slew_test.exe"
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

    def test_normal_pid_uses_gradual_release_but_open_loop_keeps_legacy_slew(self):
        source = (ROOT / "code" / "rear_motor" / "rear_motor.c").read_text(
            encoding="utf-8"
        )
        header = (ROOT / "code" / "rear_motor" / "rear_motor.h").read_text(
            encoding="utf-8"
        )

        self.assertIn("#include \"rear_motor/rear_pwm_slew.h\"", source)
        self.assertIn("#define REAR_KMY_PWM_RELEASE_LIMIT  300", header)
        self.assertIn("#define REAR_KMS_PWM_RELEASE_LIMIT  1000", header)
        self.assertIn("int pwm_release_limit;", source)
        self.assertIn("rear_pwm_limit_request_sign(target_mps, requested_pwm)", source)
        self.assertIn("rear_motor_set_drive_pwm(requested_pwm);", source)
        self.assertIn("rear_motor_set_pwm(pwm);", source)
        self.assertIn("int16  rear_motor_get_requested_pwm(void);", header)


if __name__ == "__main__":
    unittest.main()
