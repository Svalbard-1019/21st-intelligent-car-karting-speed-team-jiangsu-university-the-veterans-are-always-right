from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


def function_body(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def braced_function_body(source: str, signature: str) -> str:
    start = source.index(signature + "\n{")
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start:index + 1]
    raise AssertionError(f"Unterminated function: {signature}")


class Portion3DirectReverseMathTests(unittest.TestCase):
    def test_reverse_tracker_math_on_host(self):
        program = r'''
#include <assert.h>
#include <math.h>
#include "portion3_reverse_tracker.h"

int main(void)
{
    float steering;
    float route_m[5] = {0.0f, 0.15f, 0.55f, 0.75f, 1.40f};
    portion3_reverse_turn_state_t turn_state;
    assert(fabsf(portion3_reverse_motion_heading(10.0f) + 170.0f) < 0.001f);
    assert(portion3_reverse_initial_index(6) == 1);
    assert(portion3_reverse_initial_index(2) == 0);
    assert(portion3_reverse_metric_preview_index(route_m, 1, 5, 0.50f) == 3);
    assert(portion3_reverse_metric_preview_index(route_m, 3, 5, 1.20f) == 4);
    assert(portion3_reverse_metric_preview_index(route_m, -2, 5, 0.10f) == 1);
    steering = portion3_reverse_steering(20.0f, 0.8f, 0.724f, 1.0f, 25.0f);
    assert(steering < 0.0f);
    assert(fabsf(steering) <= 25.0f);
    assert(portion3_reverse_should_stop(0.10f, 4.0f, 5, 6));
    assert(!portion3_reverse_should_stop(0.40f, 4.0f, 5, 6));
    assert(portion3_reverse_safety_stop(10.2f, 10.0f, 0.15f));
    assert(!portion3_reverse_safety_stop(10.1f, 10.0f, 0.15f));
    portion3_reverse_turn_reset(&turn_state);
    assert(portion3_reverse_turn_level(&turn_state, 14.0f) == 0u);
    assert(portion3_reverse_turn_level(&turn_state, 16.0f) == 1u);
    assert(portion3_reverse_turn_level(&turn_state, 12.0f) == 1u);
    assert(portion3_reverse_turn_level(&turn_state, 31.0f) == 2u);
    assert(portion3_reverse_turn_level(&turn_state, 27.0f) == 2u);
    assert(portion3_reverse_turn_level(&turn_state, 46.0f) == 3u);
    assert(portion3_reverse_turn_level(&turn_state, 42.0f) == 3u);
    assert(portion3_reverse_turn_level(&turn_state, 39.0f) == 2u);
    assert(fabsf(portion3_reverse_curve_speed(-20.0f, -15.0f, 0u, 0u) + 20.0f) < 0.001f);
    assert(fabsf(portion3_reverse_curve_speed(-20.0f, -15.0f, 1u, 0u) + 17.0f) < 0.001f);
    assert(fabsf(portion3_reverse_curve_speed(-20.0f, -15.0f, 2u, 0u) + 14.0f) < 0.001f);
    assert(fabsf(portion3_reverse_curve_speed(-20.0f, -15.0f, 3u, 0u) + 11.0f) < 0.001f);
    assert(fabsf(portion3_reverse_curve_speed(-20.0f, -8.0f, 3u, 1u) + 8.0f) < 0.001f);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as temp_dir:
            temp = Path(temp_dir)
            source = temp / "portion3_reverse_test.c"
            binary = temp / "portion3_reverse_test.exe"
            source.write_text(program, encoding="utf-8")
            compile_result = subprocess.run(
                [
                    "gcc",
                    "-std=c99",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-I",
                    str(ROOT / "code"),
                    str(source),
                    "-lm",
                    "-o",
                    str(binary),
                ],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                check=False,
            )
            self.assertEqual(compile_result.returncode, 0, compile_result.stderr)
            run_result = subprocess.run(
                [str(binary)],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                check=False,
            )
            self.assertEqual(run_result.returncode, 0, run_result.stderr)


class Portion3DirectReverseIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.guandao_c = (ROOT / "code" / "guandao.c").read_text(encoding="utf-8")
        cls.guandao_h = (ROOT / "code" / "guandao.h").read_text(encoding="utf-8")
        cls.main_c = (ROOT / "user" / "cpu0_main.c").read_text(encoding="utf-8")

    def test_save_prepares_direct_reverse_after_flash(self):
        recode_body = function_body(
            self.guandao_c,
            "void guandao_recode(guandao_state * state)",
            "void guandao_record_session_reset(void)",
        )
        self.assertIn("portion3_direct_reverse_prepare(p);", recode_body)
        self.assertLess(
            recode_body.index("Flash_Store_Mode(route_setting_choice);"),
            recode_body.index("portion3_direct_reverse_prepare(p);"),
        )
        self.assertNotIn("portion3_return_reset();", recode_body)
        self.assertIn("state_t direct_current_state;", recode_body)
        self.assertIn("direct_current_state = p->current_state;", recode_body)
        self.assertIn("p->current_state = direct_current_state;", recode_body)
        self.assertIn("guandao_planned_map[i] = p->recode_map[i];", recode_body)
        self.assertIn("p->recode_map[i] = guandao_planned_map[i];", recode_body)
        self.assertLess(
            recode_body.index("guandao_planned_map[i] = p->recode_map[i];"),
            recode_body.index("Flash_Store_Mode(route_setting_choice);"),
        )
        self.assertLess(
            recode_body.index("Flash_Store_Mode(route_setting_choice);"),
            recode_body.index("p->recode_map[i] = guandao_planned_map[i];"),
        )
        self.assertLess(
            recode_body.index("p->recode_map[i] = guandao_planned_map[i];"),
            recode_body.index("portion3_direct_reverse_prepare(p);"),
        )

    def test_prepare_preserves_pose_and_enters_reverse_mode(self):
        signature = "static uint8 portion3_direct_reverse_prepare(guandao_state *route)"
        self.assertIn(signature, self.guandao_c)
        prepare_body = braced_function_body(self.guandao_c, signature)
        self.assertIn("main_mode = Guandao_portion_3;", prepare_body)
        self.assertIn("conrtol_mode = DAOCHE;", prepare_body)
        self.assertIn("daoche_flag = 1;", prepare_body)
        self.assertIn("route->recode_map[length - 1 - i]", prepare_body)
        self.assertNotIn("rear_motor_reset_odometry", prepare_body)
        self.assertNotIn("IMU_yaw_rezero_start", prepare_body)
        self.assertNotIn("current_state.x = 0", prepare_body)
        self.assertNotIn("Yaw_1 = 0", prepare_body)

    def test_trace_routes_active_direct_reverse_before_rezero(self):
        trace_body = function_body(
            self.guandao_c,
            "void guandao_trace(guandao_state * state)",
            "float speed_calculate(",
        )
        self.assertIn("if(portion3_direct_reverse_active())", trace_body)
        self.assertIn("portion3_direct_reverse_update();", trace_body)
        self.assertLess(
            trace_body.index("if(portion3_direct_reverse_active())"),
            trace_body.index("if(route_setting_choice == 2 && portion3_start_rezero_pending)"),
        )

    def test_reverse_update_has_geometric_stops_without_time_limit(self):
        signature = "static void portion3_direct_reverse_update(void)"
        self.assertIn(signature, self.guandao_c)
        update_body = braced_function_body(self.guandao_c, signature)
        self.assertIn("update_state(&portion_3, &guandao_ecd);", update_body)
        self.assertIn("portion3_reverse_motion_heading(Yaw_1)", update_body)
        self.assertIn("portion3_reverse_steering(", update_body)
        self.assertIn("daoche_flag = 1;", update_body)
        self.assertIn("conrtol_mode = DAOCHE;", update_body)
        self.assertIn("rear_motor_brake_start();", update_body)
        self.assertNotIn("PORTION3_DIRECT_REVERSE_MAX_MS", self.guandao_c)
        self.assertNotIn("portion3_direct_reverse_start_ms", self.guandao_c)
        self.assertNotIn("portion3_reverse_safety_stop(", update_body)
        self.assertIn("portion3_reverse_should_stop(", update_body)
        self.assertIn("guandao_portion3_terminal_passed(", update_body)
        self.assertNotIn("portion3_direct_reverse_travelled", self.guandao_c)
        self.assertNotIn("portion3_direct_reverse_route_length", self.guandao_c)

    def test_reverse_uses_portion1_metric_lookahead_and_curve_speed(self):
        signature = "static void portion3_direct_reverse_update(void)"
        update_body = braced_function_body(self.guandao_c, signature)
        self.assertNotIn("PORTION3_DIRECT_REVERSE_PREVIEW_STEPS", self.guandao_c)
        self.assertIn("guandao_reverse_lookahead_m(reference_speed)", update_body)
        self.assertIn("portion3_reverse_metric_preview_index(", update_body)
        self.assertIn("portion3_reverse_turn_level(", update_body)
        self.assertIn("portion3_reverse_curve_speed(", update_body)

    def test_reverse_prepare_builds_arc_cache_after_ram_route_reversal(self):
        signature = "static uint8 portion3_direct_reverse_prepare(guandao_state *route)"
        body = braced_function_body(self.guandao_c, signature)
        reverse = body.index(
            "route->recode_map[i] = route->recode_map[length - 1 - i];"
        )
        cache = body.index("portion3_direct_reverse_route_m[i]")
        self.assertLess(reverse, cache)
        self.assertIn(
            "get_distance(route->recode_map[i - 1], route->recode_map[i])",
            body,
        )

    def test_adaptive_target_does_not_replace_pose_or_index_updates(self):
        signature = "static void portion3_direct_reverse_update(void)"
        body = braced_function_body(self.guandao_c, signature)
        self.assertEqual(body.count("update_state(&portion_3, &guandao_ecd);"), 1)
        self.assertEqual(
            body.count(
                "portion_3.current_point_index = "
                "portion3_direct_reverse_route_index;"
            ),
            1,
        )
        target_call = body.split(
            "portion3_reverse_metric_preview_index(", 1
        )[1].split(");", 1)[0]
        self.assertNotIn("current_state =", target_call)
        self.assertNotIn("current_point_index =", target_call)

    def test_reverse_does_not_exceed_portion1_forward_steering_limit(self):
        signature = "static void portion3_direct_reverse_update(void)"
        update_body = braced_function_body(self.guandao_c, signature)

        self.assertIn(
            "#define PORTION3_DIRECT_REVERSE_STEER_LIMIT GUANDAO_VERY_HIGH_CMD_LIMIT",
            self.guandao_c,
        )
        self.assertIn(
            "PORTION3_DIRECT_REVERSE_GAIN, PORTION3_DIRECT_REVERSE_STEER_LIMIT",
            update_body,
        )
        self.assertIn(
            "-PORTION3_DIRECT_REVERSE_STEER_LIMIT,\n"
            "                PORTION3_DIRECT_REVERSE_STEER_LIMIT",
            update_body,
        )

    def test_diagnostics_identify_direct_reverse_firmware(self):
        self.assertIn("P3AUTO,cfg=p3rev1", self.main_c)
        for field in (
            "p3Rev=%u",
            "p3Stop=%u",
            "revIdx=%d",
            "revD100=%ld",
            "revCmd10=%ld",
            "revLd100=%ld",
            "revTurn=%u",
            "revSpd10=%ld",
        ):
            self.assertIn(field, self.main_c)
        for declaration in (
            "uint8 guandao_portion3_reverse_active(void);",
            "uint8 guandao_portion3_reverse_stop_cause(void);",
            "int16 guandao_portion3_reverse_index(void);",
            "float guandao_portion3_reverse_final_distance(void);",
            "float guandao_portion3_reverse_steer_command(void);",
            "float guandao_portion3_reverse_lookahead(void);",
            "uint8 guandao_portion3_reverse_turn_level(void);",
            "float guandao_portion3_reverse_speed_command(void);",
        ):
            self.assertIn(declaration, self.guandao_h)


if __name__ == "__main__":
    unittest.main()
