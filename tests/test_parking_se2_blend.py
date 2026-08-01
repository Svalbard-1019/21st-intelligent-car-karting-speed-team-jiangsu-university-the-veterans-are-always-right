from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ParkingSe2BlendTests(unittest.TestCase):
    def test_runtime_anchor_blends_smoothly_into_fixed_taught_target(self):
        source = textwrap.dedent(
            r"""
            #include <math.h>
            #include "parking_se2.h"

            static int close_enough(float actual, float expected)
            {
                return fabsf(actual - expected) < 0.0001f;
            }

            int main(void)
            {
                float x;
                float y;
                float heading;

                parking_se2_blend_pose_to_fixed(
                        1.2f, -0.4f, 10.0f,
                        0.0f, 0.0f, 30.0f,
                        0.0f, &x, &y, &heading);
                if(!close_enough(x, 1.2f) || !close_enough(y, -0.4f)
                        || !close_enough(heading, 10.0f)) return 1;

                parking_se2_blend_pose_to_fixed(
                        1.2f, -0.4f, 10.0f,
                        0.0f, 0.0f, 30.0f,
                        0.5f, &x, &y, &heading);
                if(!close_enough(x, 0.6f) || !close_enough(y, -0.2f)
                        || !close_enough(heading, 20.0f)) return 2;

                parking_se2_blend_pose_to_fixed(
                        1.2f, -0.4f, 10.0f,
                        0.0f, 0.0f, 30.0f,
                        1.0f, &x, &y, &heading);
                if(!close_enough(x, 0.0f) || !close_enough(y, 0.0f)
                        || !close_enough(heading, 30.0f)) return 3;

                parking_se2_blend_pose_to_fixed(
                        0.0f, 0.0f, 170.0f,
                        0.0f, 0.0f, -170.0f,
                        0.5f, &x, &y, &heading);
                if(!close_enough(fabsf(heading), 180.0f)) return 4;
                return 0;
            }
            """
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            source_path = temp_path / "parking_se2_blend_test.c"
            executable_path = temp_path / "parking_se2_blend_test.exe"
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


if __name__ == "__main__":
    unittest.main()
