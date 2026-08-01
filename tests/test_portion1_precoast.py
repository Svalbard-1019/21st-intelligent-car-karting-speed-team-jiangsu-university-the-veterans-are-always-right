from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest


ROOT = Path(__file__).resolve().parents[1]


class Portion1PreCoastPolicyTests(unittest.TestCase):
    def test_policy_formula_thresholds_and_latch(self):
        source = textwrap.dedent(
            r"""
            #include <math.h>
            #include "portion1_precoast.h"

            static int near(float a, float b)
            {
                return fabsf(a - b) < 0.001f;
            }

            int main(void)
            {
                portion1_precoast_t state;
                portion1_precoast_reset(&state);
                if(!near(portion1_precoast_trigger_m(2.74f), 6.4713f)) return 1;
                portion1_precoast_update(&state, 1u, 1u, 30.0f, 2.74f, 6.0f);
                if(state.latched) return 2;
                portion1_precoast_update(&state, 0u, 1u, 40.0f, 2.74f, 6.0f);
                if(state.latched) return 3;
                portion1_precoast_update(&state, 1u, 1u, 40.0f, 2.74f, 6.0f);
                if(!state.latched || !state.coast_output) return 4;
                portion1_precoast_update(&state, 1u, 1u, 40.0f, 1.10f, 8.0f);
                if(!state.latched || state.coast_output) return 5;
                portion1_precoast_update(&state, 1u, 0u, 40.0f, 2.00f, 2.0f);
                if(state.latched || state.coast_output) return 6;
                return 0;
            }
            """
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            source_path = temp_path / "precoast_test.c"
            executable_path = temp_path / "precoast_test.exe"
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
