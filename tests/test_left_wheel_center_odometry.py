from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest


ROOT = Path(__file__).resolve().parents[1]


class LeftWheelCenterOdometryTests(unittest.TestCase):
    def test_left_wheel_distance_is_converted_to_vehicle_center_distance(self):
        source = textwrap.dedent(
            r"""
            #include <math.h>
            #include "rear_motor/rear_left_wheel_odometry.h"

            static int close_enough(float actual, float expected)
            {
                return fabsf(actual - expected) < 0.0001f;
            }

            int main(void)
            {
                rear_left_wheel_odometry_t odometry;
                const float track_width = 0.594f;
                const float half_turn_offset = track_width * 3.14159265358979323846f / 4.0f;
                float center_distance;

                rear_left_wheel_odometry_reset(&odometry);
                center_distance = rear_left_wheel_to_center_distance(
                        &odometry, 1.0f, 0.0f, track_width);
                if(!close_enough(center_distance, 1.0f)) return 1;

                center_distance = rear_left_wheel_to_center_distance(
                        &odometry, 1.0f + half_turn_offset, 90.0f, track_width);
                if(!close_enough(center_distance, 1.0f)) return 2;

                rear_left_wheel_odometry_reset(&odometry);
                (void)rear_left_wheel_to_center_distance(
                        &odometry, 0.0f, 0.0f, track_width);
                center_distance = rear_left_wheel_to_center_distance(
                        &odometry, 1.0f - half_turn_offset, -90.0f, track_width);
                if(!close_enough(center_distance, 1.0f)) return 3;

                rear_left_wheel_odometry_reset(&odometry);
                (void)rear_left_wheel_to_center_distance(
                        &odometry, 0.0f, 179.0f, track_width);
                center_distance = rear_left_wheel_to_center_distance(
                        &odometry,
                        1.0f + track_width * 3.14159265358979323846f / 180.0f,
                        -179.0f,
                        track_width);
                if(!close_enough(center_distance, 1.0f)) return 4;

                return 0;
            }
            """
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            source_path = temp_path / "left_wheel_center_test.c"
            executable_path = temp_path / "left_wheel_center_test.exe"
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

    def test_guandao_uses_and_resets_center_odometry(self):
        source = (ROOT / "code" / "guandao.c").read_text(encoding="utf-8")

        self.assertIn(
            '#include "rear_motor/rear_left_wheel_odometry.h"',
            source,
        )
        self.assertIn("rear_left_wheel_to_center_distance(", source)
        self.assertIn("state == &portion_3", source)
        self.assertGreaterEqual(
            source.count("rear_left_wheel_odometry_reset("),
            2,
        )


if __name__ == "__main__":
    unittest.main()
