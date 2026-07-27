from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


def function_body(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


class Portion3StartRezeroTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.imu_c = (ROOT / "code" / "IMU.c").read_text(encoding="utf-8")
        cls.imu_h = (ROOT / "code" / "IMU.h").read_text(encoding="utf-8")
        cls.rear_c = (
            ROOT / "code" / "rear_motor" / "rear_motor.c"
        ).read_text(encoding="utf-8")
        cls.rear_h = (
            ROOT / "code" / "rear_motor" / "rear_motor.h"
        ).read_text(encoding="utf-8")
        cls.guandao_c = (ROOT / "code" / "guandao.c").read_text(encoding="utf-8")
        cls.main_c = (ROOT / "user" / "cpu0_main.c").read_text(encoding="utf-8")

    def test_imu_rezero_is_nonblocking_and_updates_offset(self):
        self.assertIn("#define IMU_YAW_REZERO_SAMPLES", self.imu_c)
        self.assertIn("void IMU_yaw_rezero_start(void)", self.imu_c)
        self.assertIn("uint8 IMU_yaw_rezero_active(void)", self.imu_c)
        self.assertIn("int16 IMU_yaw_rezero_raw_z(void)", self.imu_c)
        self.assertIn("float IMU_yaw_rezero_offset_z(void)", self.imu_c)
        self.assertIn("void IMU_yaw_rezero_start(void);", self.imu_h)

        start_body = function_body(
            self.imu_c,
            "void IMU_yaw_rezero_start(void)",
            "uint8 IMU_yaw_rezero_active(void)",
        )
        get_body = function_body(
            self.imu_c,
            "void IMU_GetValues(void)",
            "void IMU_Handle_180(void)",
        )
        self.assertNotIn("system_delay_ms", start_body)
        self.assertIn("imu_yaw_rezero_count >= IMU_YAW_REZERO_SAMPLES", get_body)
        self.assertIn(
            "Gyro_Offset.Zdata = (float)imu_yaw_rezero_sum",
            get_body,
        )
        self.assertLess(
            get_body.index("if(imu_yaw_rezero_running)"),
            get_body.index("IMU_Data.gyro_z ="),
        )

    def test_rear_odometry_reset_clears_queue_and_syncs_encoder(self):
        self.assertIn("void rear_motor_reset_odometry(void);", self.rear_h)
        self.assertIn("void rear_motor_reset_odometry(void)", self.rear_c)
        reset_body = function_body(
            self.rear_c,
            "void rear_motor_reset_odometry(void)",
            "uint8 rear_motor_take_odometry_sample(",
        )
        self.assertIn(
            "rear_odometry_pose_buffer_init(&odometry_pose_buffer);",
            reset_body,
        )
        self.assertIn("odometry_total_pulses = 0;", reset_body)
        self.assertIn(
            "last_encoder_count = encoder_get_count(TIM2_ENCODER);",
            reset_body,
        )
        self.assertIn("interrupt_global_disable()", reset_body)
        self.assertIn("interrupt_global_enable(interrupt_state)", reset_body)

    def test_portion3_waits_for_rezero_before_tracking(self):
        trace_body = function_body(
            self.guandao_c,
            "void guandao_trace(guandao_state * state)",
            "float speed_calculate(",
        )
        reset_body = function_body(
            self.guandao_c,
            "void portion3_return_reset(void)",
            "void Guandao_Points_Show(",
        )
        self.assertIn("static uint8 portion3_start_rezero_pending = 0;", self.guandao_c)
        self.assertIn("IMU_yaw_rezero_start();", reset_body)
        self.assertIn("rear_motor_reset_odometry();", reset_body)
        self.assertIn("if(IMU_yaw_rezero_active()) return;", trace_body)
        self.assertIn("guandao_debug_stop_reason = 12;", trace_body)
        self.assertLess(
            trace_body.index("if(route_setting_choice == 2"),
            trace_body.index("update_state(p,&guandao_ecd);"),
        )
        self.assertGreaterEqual(trace_body.count("rear_motor_reset_odometry();"), 1)

    def test_portion3_diagnostics_expose_startup_rezero(self):
        self.assertIn("P3AUTO,cfg=p3save2", self.main_c)
        self.assertIn("p3Init=%u", self.main_c)
        self.assertIn("gzRaw=%d", self.main_c)
        self.assertIn("gzOff=%ld", self.main_c)
        self.assertIn("IMU_yaw_rezero_active()", self.main_c)
        self.assertIn("IMU_yaw_rezero_raw_z()", self.main_c)
        self.assertIn("IMU_yaw_rezero_offset_z()", self.main_c)


if __name__ == "__main__":
    unittest.main()
