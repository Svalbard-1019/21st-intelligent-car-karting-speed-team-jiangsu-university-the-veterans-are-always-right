from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest


ROOT = Path(__file__).resolve().parents[1]


class Portion1ParkingBrakeTests(unittest.TestCase):
    def test_speed_dependent_slowdown_window(self):
        source = textwrap.dedent(
            r"""
            #include "portion1_parking_brake.h"

            int main(void)
            {
                float slow_distance = portion1_parking_slowdown_distance(2.0f, 1.0f);
                float fast_distance = portion1_parking_slowdown_distance(3.0f, 1.0f);

                if(!(fast_distance > slow_distance)) return 1;
                if(!portion1_parking_should_slowdown(
                        -fast_distance + 0.01f, 3.0f, 1.0f, 0u)) return 2;
                if(portion1_parking_should_slowdown(
                        -fast_distance - 0.05f, 3.0f, 1.0f, 0u)) return 3;
                if(portion1_parking_should_slowdown(
                        -0.50f, 1.2f, 1.0f, 0u)) return 4;
                if(portion1_parking_should_slowdown(
                        0.05f, 3.0f, 1.0f, 0u)) return 5;
                if(portion1_parking_should_slowdown(
                        -0.50f, 3.0f, 1.0f, 1u)) return 6;
                return 0;
            }
            """
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            source_path = temp_path / "portion1_parking_brake_test.c"
            executable_path = temp_path / "portion1_parking_brake_test.exe"
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

    def test_portion1_approach_requests_one_shot_slowdown(self):
        source = (ROOT / "code" / "guandao.c").read_text(encoding="utf-8")
        portion_one = source.split("void portion_1(void)", 1)[1]
        portion_one = portion_one.split("void recode_waypoint(", 1)[0]

        self.assertIn('#include "portion1_parking_brake.h"', source)
        self.assertIn("portion1_approach_brake_requested", source)
        self.assertIn("rear_motor_get_speed_mps()", portion_one)
        self.assertIn("portion1_parking_should_slowdown(", portion_one)
        self.assertIn("rear_motor_brake_to_speed_start(", portion_one)

    def test_final_stop_can_upgrade_active_slowdown_brake(self):
        source = (ROOT / "code" / "rear_motor" / "rear_motor.c").read_text(
            encoding="utf-8"
        )
        header = (ROOT / "code" / "rear_motor" / "rear_motor.h").read_text(
            encoding="utf-8"
        )
        start = source.split("void rear_motor_brake_start(void)", 1)[1]
        start = start.split("void rear_motor_brake_update(void)", 1)[0]

        self.assertIn("void rear_motor_brake_to_speed_start(float target_speed_mps);", header)
        self.assertIn("brake_target_speed_mps", source)
        self.assertIn("if(brake_active)", start)
        self.assertIn("brake_target_speed_mps = 0.0f;", start)


if __name__ == "__main__":
    unittest.main()
