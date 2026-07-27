# Portion 3 Start Rezero Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a direct Portion 3 start establish the same gyro and odometry runtime origin as an MCU-reset start.

**Architecture:** Extend the existing IMU ISR update path with a 125-sample nonblocking Z-axis rezero state. Gate Portion 3 tracking until that state finishes, then atomically reset the rear odometry queue and encoder baseline before consuming route points. Expose the startup state in `P3AUTO` diagnostics.

**Tech Stack:** Infineon AURIX C firmware, ISR-driven IMU sampling, Python `unittest` source regression tests, Git.

---

### Task 1: Add failing startup-state regression tests

**Files:**
- Create: `tests/test_portion3_start_rezero.py`

- [ ] **Step 1: Write the failing tests**

Create source tests that assert the intended interfaces and control order:

```python
self.assertIn("#define IMU_YAW_REZERO_SAMPLES", imu_c)
self.assertIn("void IMU_yaw_rezero_start(void)", imu_c)
self.assertIn("uint8 IMU_yaw_rezero_active(void)", imu_c)
self.assertNotIn("system_delay_ms", rezero_start_body)
self.assertIn("Gyro_Offset.Zdata = (float)imu_yaw_rezero_sum", imu_get_body)
self.assertIn("void rear_motor_reset_odometry(void)", rear_c)
self.assertIn("rear_odometry_pose_buffer_init(&odometry_pose_buffer);", reset_body)
self.assertIn("last_encoder_count = encoder_get_count(TIM2_ENCODER);", reset_body)
self.assertLess(trace_body.index("IMU_yaw_rezero_active()"),
                trace_body.index("update_state(p,&guandao_ecd);"))
self.assertIn("guandao_debug_stop_reason = 12;", trace_body)
self.assertIn("P3AUTO,cfg=p3save2", main_c)
```

- [ ] **Step 2: Run the focused tests**

Run: `python -m unittest tests.test_portion3_start_rezero -v`

Expected: failures because none of the new APIs or diagnostics exist.

- [ ] **Step 3: Commit the red tests**

```text
git add tests/test_portion3_start_rezero.py
git commit -m "test: cover portion3 startup rezero"
```

### Task 2: Add the nonblocking IMU rezero state

**Files:**
- Modify: `code/IMU.c`
- Modify: `code/IMU.h`
- Test: `tests/test_portion3_start_rezero.py`

- [ ] **Step 1: Declare the public API**

Add:

```c
void IMU_yaw_rezero_start(void);
uint8 IMU_yaw_rezero_active(void);
int16 IMU_yaw_rezero_raw_z(void);
float IMU_yaw_rezero_offset_z(void);
```

- [ ] **Step 2: Implement the state**

Add a 125-sample accumulator. `IMU_yaw_rezero_start()` atomically clears its sum/count, marks it active, and zeros `Yaw_1`. At the start of `IMU_GetValues()`, active calibration accumulates `imu963ra_gyro_z`, holds yaw and physical gyro output at zero, updates `Gyro_Offset.Zdata` after the 125th sample, and returns before normal integration.

- [ ] **Step 3: Run the IMU-focused tests**

Run: `python -m unittest tests.test_portion3_start_rezero.Portion3StartRezeroTests.test_imu_rezero_is_nonblocking_and_updates_offset -v`

Expected: 1 test passes.

### Task 3: Add an atomic rear odometry reset

**Files:**
- Modify: `code/rear_motor/rear_motor.c`
- Modify: `code/rear_motor/rear_motor.h`
- Test: `tests/test_portion3_start_rezero.py`

- [ ] **Step 1: Add the public reset**

Implement:

```c
void rear_motor_reset_odometry(void)
{
    uint32 interrupt_state = interrupt_global_disable();
    rear_odometry_pose_buffer_init(&odometry_pose_buffer);
    odometry_total_pulses = 0;
    encoder_10ms = 0;
    last_encoder_count = encoder_get_count(TIM2_ENCODER);
    encoder_first_read = 0;
    interrupt_global_enable(interrupt_state);
}
```

- [ ] **Step 2: Run the odometry-focused test**

Run: `python -m unittest tests.test_portion3_start_rezero.Portion3StartRezeroTests.test_rear_odometry_reset_clears_queue_and_syncs_encoder -v`

Expected: 1 test passes.

### Task 4: Gate Portion 3 until the new origin is ready

**Files:**
- Modify: `code/guandao.c`
- Test: `tests/test_portion3_start_rezero.py`

- [ ] **Step 1: Start calibration from the Portion 3 reset**

`portion3_return_reset()` calls `rear_motor_reset_odometry()` and `IMU_yaw_rezero_start()`, then records that the startup calibration is pending.

- [ ] **Step 2: Hold tracking while active**

Before `update_state()` in `guandao_trace()`:

```c
if(route_setting_choice == 2 && portion3_start_rezero_pending)
{
    out_v_l = 0.0f;
    out_v_r = 0.0f;
    out_servo = 0.0f;
    guandao_debug_stop_reason = 12;
    if(IMU_yaw_rezero_active()) return;
    rear_motor_reset_odometry();
    rear_left_wheel_odometry_reset(&portion3_center_odometry);
    p->current_state.x = 0.0f;
    p->current_state.y = 0.0f;
    p->current_state.theta = 0.0f;
    p->current_point_index = 0;
    portion3_start_rezero_pending = 0;
}
```

- [ ] **Step 3: Run the startup-gate test**

Run: `python -m unittest tests.test_portion3_start_rezero.Portion3StartRezeroTests.test_portion3_waits_for_rezero_before_tracking -v`

Expected: 1 test passes.

### Task 5: Add diagnostics and verify

**Files:**
- Modify: `user/cpu0_main.c`
- Modify: `tests/test_final_rear_interface.py`
- Test: `tests/test_portion3_start_rezero.py`

- [ ] **Step 1: Update the diagnostic format**

Change the marker to `p3save2` and add:

```text
p3Init=%u,gzRaw=%d,gzOff=%ld
```

Supply `IMU_yaw_rezero_active()`, `IMU_yaw_rezero_raw_z()`, and the scaled offset as matching `sprintf` arguments.

- [ ] **Step 2: Run the full suite**

Run: `python -m unittest discover -s tests -p 'test_*.py'`

Expected: all tests pass with zero failures.

- [ ] **Step 3: Review and commit**

Run `git diff --check`, inspect the scoped diff, then commit:

```text
git add code/IMU.c code/IMU.h code/rear_motor/rear_motor.c code/rear_motor/rear_motor.h code/guandao.c user/cpu0_main.c tests/test_final_rear_interface.py
git commit -m "fix: rezero portion3 runtime state before start"
```

- [ ] **Step 4: Integrate**

Fast-forward `final`, rerun the full tests in the main worktree, and push `origin/final`.
