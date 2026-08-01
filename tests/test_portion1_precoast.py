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


    def test_rear_coast_updates_speed_without_resetting_odometry(self):
        source = (ROOT / "code/rear_motor/rear_motor.c").read_text(
            encoding="utf-8"
        )
        header = (ROOT / "code/rear_motor/rear_motor.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("void rear_motor_coast_update(void);", header)
        body = source.split("void rear_motor_coast_update(void)", 1)[1]
        body = body.split("\n}\n", 1)[0]
        self.assertIn("rear_motor_take_speed_windows", body)
        self.assertIn("rear_motor_filter_speed", body)
        self.assertIn("rear_motor_set_pwm(0);", body)
        self.assertNotIn("rear_motor_stop", body)
        for forbidden in (
            "encoder_10ms = 0",
            "speed_window_build_pulses = 0",
            "speed_window_ready_pulses = 0",
            "odometry_total_pulses = 0",
            "rear_odometry_pose_buffer_init",
        ):
            self.assertNotIn(forbidden, body)


    def test_precoast_is_portion1_forward_only_and_caps_motor_target(self):
        guandao = (ROOT / "code/guandao.c").read_text(encoding="utf-8")
        main = (ROOT / "user/cpu0_main.c").read_text(encoding="utf-8")
        dispatch = main.split(
            "static void Guandao_Rear_Motor_Update(void)", 1
        )[1].split("static int32 Serial_Debug_Scale", 1)[0]
        self.assertIn("main_mode == Guandao_portion_1", dispatch)
        self.assertIn("conrtol_mode == GUANDAO", dispatch)
        self.assertIn("guandao_portion1_precoast_output()", dispatch)
        self.assertIn("rear_motor_coast_update();", dispatch)
        self.assertIn("PORTION1_PRECOAST_TARGET_MPS", dispatch)
        self.assertLess(
            dispatch.index("rear_motor_brake_active()"),
            dispatch.index("guandao_portion1_precoast_output()"),
        )
        self.assertIn(
            "portion1_precoast_update(&portion1_precoast", guandao
        )

    def test_precoast_does_not_write_inertial_or_steering_state(self):
        source = (ROOT / "code/guandao.c").read_text(encoding="utf-8")
        self.assertIn("static void guandao_portion1_precoast_update", source)
        block = source.split(
            "static void guandao_portion1_precoast_update", 1
        )[1].split(
            "\n}\n\nuint8 guandao_portion1_precoast_latched", 1
        )[0]
        for forbidden in (
            "INS.current_state.x =",
            "INS.current_state.y =",
            "INS.current_state.theta =",
            "INS.current_point_index =",
            "out_servo =",
            "pursuit_contral_mode(",
            "update_state(",
        ):
            self.assertNotIn(forbidden, block)

    def test_route_remaining_cache_is_built_once_and_tracking_read_is_const(self):
        source = (ROOT / "code/guandao.c").read_text(encoding="utf-8")
        self.assertIn("static void guandao_portion1_precoast_prepare", source)
        self.assertIn("static float guandao_portion1_remaining_m", source)
        prepare = source.split(
            "static void guandao_portion1_precoast_prepare", 1
        )[1].split("static float guandao_portion1_remaining_m", 1)[0]
        remaining = source.split(
            "static float guandao_portion1_remaining_m", 1
        )[1].split("static void guandao_portion1_precoast_update", 1)[0]
        self.assertIn("portion1_route_remaining_m[i]", prepare)
        self.assertIn("get_distance", prepare)
        self.assertIn(
            "get_distance(INS.current_state, INS.recode_map[index])",
            remaining,
        )
        self.assertNotIn("INS.current_point_index =", remaining)
        self.assertNotIn("INS.current_state.x =", remaining)
        self.assertNotIn("INS.current_state.y =", remaining)
        self.assertNotIn("INS.current_state.theta =", remaining)


    def test_serial_exposes_precoast_diagnostics(self):
        source = (ROOT / "user/cpu0_main.c").read_text(encoding="utf-8")
        start = source.index('"AUTO,')
        end = source.index("\\r\\n", start)
        auto_format = source[start:end]
        for field in (
            "preCoast=%u",
            "remain100=%ld",
            "coastTrig100=%ld",
            "coastOut=%u",
        ):
            self.assertIn(field, auto_format)
        for getter in (
            "guandao_portion1_precoast_latched()",
            "guandao_portion1_precoast_remaining_m()",
            "guandao_portion1_precoast_trigger_m()",
            "guandao_portion1_precoast_output()",
        ):
            self.assertIn(getter, source)


if __name__ == "__main__":
    unittest.main()
