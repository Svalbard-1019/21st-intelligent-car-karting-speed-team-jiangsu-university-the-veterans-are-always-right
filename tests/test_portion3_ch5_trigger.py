from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class RemoteOneShotTests(unittest.TestCase):
    def test_press_fires_once_until_release(self):
        program = r"""
#include <assert.h>
#include "remote_one_shot.h"

int main(void)
{
    remote_one_shot_t trigger;
    remote_one_shot_reset(&trigger);
    assert(remote_one_shot_update(&trigger, 0u) == 0u);
    assert(remote_one_shot_update(&trigger, 1u) == 1u);
    assert(remote_one_shot_update(&trigger, 1u) == 0u);
    assert(remote_one_shot_update(&trigger, 1u) == 0u);
    assert(remote_one_shot_update(&trigger, 0u) == 0u);
    assert(remote_one_shot_update(&trigger, 1u) == 1u);
    return 0;
}
"""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp = Path(temp_dir)
            source = temp / "remote_one_shot_test.c"
            binary = temp / "remote_one_shot_test.exe"
            source.write_text(program, encoding="utf-8")
            compile_result = subprocess.run(
                [
                    "gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                    "-I", str(ROOT / "code"), str(source), "-o", str(binary),
                ],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                check=False,
            )
            self.assertEqual(compile_result.returncode, 0, compile_result.stderr)
            run_result = subprocess.run(
                [str(binary)], capture_output=True, text=True,
                encoding="utf-8", errors="replace", check=False,
            )
            self.assertEqual(run_result.returncode, 0, run_result.stderr)


class Portion3Ch5IntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.remote_c = (ROOT / "code" / "RemteControl.c").read_text(encoding="utf-8")
        cls.guandao_c = (ROOT / "code" / "guandao.c").read_text(encoding="utf-8")
        cls.guandao_h = (ROOT / "code" / "guandao.h").read_text(encoding="utf-8")
        cls.main_c = (ROOT / "user" / "cpu0_main.c").read_text(encoding="utf-8")

    def test_sbus_ch5_is_mapped_and_fails_safe_released(self):
        self.assertIn("uint16 ch_reverse;", self.remote_c)
        self.assertIn("ch_reverse = uart_receiver.channel[4];", self.remote_c)
        self.assertIn("x6f_out[4] = (ch_reverse > 1500u) ? 200 : 100;", self.remote_c)
        control_start = self.remote_c.index("void sbus_rc_control(void)")
        receiver_lost = self.remote_c.index("if(uart_receiver.state == 0)", control_start)
        receiver_return = self.remote_c.index("return;", receiver_lost)
        self.assertIn("x6f_out[4] = 100;", self.remote_c[receiver_lost:receiver_return])

    def test_ch5_starts_brake_then_existing_save_reverse_pipeline(self):
        start = self.guandao_c.index("void guandao_recode(guandao_state * state)")
        end = self.guandao_c.index("void guandao_record_session_reset(void)", start)
        body = self.guandao_c[start:end]
        trigger = body.index("remote_one_shot_update(&portion3_ch5_trigger")
        pending = body.index("if(portion3_save_pending)")
        self.assertLess(trigger, pending)
        trigger_block = body[trigger:pending]
        self.assertIn("x6f_out[4] == 200", trigger_block)
        self.assertIn("route_setting_choice == 2", trigger_block)
        self.assertIn("p == &portion_3", trigger_block)
        self.assertIn("p->length_index > 1", trigger_block)
        self.assertIn("!portion3_save_pending", trigger_block)
        self.assertIn("!guandao_record_saved", trigger_block)
        self.assertIn("portion3_save_pending = 1;", trigger_block)
        self.assertIn("rear_motor_brake_start();", trigger_block)
        self.assertNotIn("portion3_direct_reverse_prepare", trigger_block)

    def test_trigger_is_reset_with_record_session(self):
        self.assertIn('#include "remote_one_shot.h"', self.guandao_c)
        reset_start = self.guandao_c.index("void guandao_record_session_reset(void)")
        reset_body = self.guandao_c[reset_start:reset_start + 900]
        self.assertIn("remote_one_shot_reset(&portion3_ch5_trigger);", reset_body)
        self.assertIn("portion3_ch5_triggered = 0u;", reset_body)

    def test_record_diagnostics_expose_ch5_trigger(self):
        self.assertIn("ch5=%d,p3Trig=%u", self.main_c)
        self.assertIn("x6f_out[4]", self.main_c)
        self.assertIn("guandao_portion3_ch5_triggered()", self.main_c)
        self.assertIn("uint8 guandao_portion3_ch5_triggered(void);", self.guandao_h)


if __name__ == "__main__":
    unittest.main()
