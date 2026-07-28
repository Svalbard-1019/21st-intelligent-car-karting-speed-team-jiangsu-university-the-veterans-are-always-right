# Portion3 Direct Reverse Return Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** After a Portion3 route is saved, automatically reverse along that same route to its original start without a reset, button press, or turn-around.

**Architecture:** Preserve the existing Flash payload format by temporarily backing up the recorded route and live endpoint pose in the existing planning buffer while `Flash_Store_Mode(2)` performs its legacy conversion. Restore the recording frame afterward, reverse only that RAM route, and run a Portion3-only reverse state machine using `Yaw_1 + 180°`, limited reverse steering, approach slowdown, and distance/time safety stops.

**Tech Stack:** Infineon TC264 C, existing rear-motor/IMU/route modules, Python `unittest`, host GCC tests for isolated math helpers.

---

## File map

- Create `code/portion3_reverse_tracker.h`: pure, hardware-free reverse heading, steering, progress, and stop-gate helpers.
- Modify `code/guandao.c`: save-to-reverse transition, RAM route reversal, Portion3 reverse state machine, braking and diagnostics accessors.
- Modify `code/guandao.h`: public Portion3 reverse diagnostic declarations.
- Modify `user/cpu0_main.c`: P3 diagnostic fields and firmware marker.
- Create `tests/test_portion3_direct_reverse.py`: host math tests and source-contract integration tests.
- Modify `tests/test_portion3_save_consistency.py`, `tests/test_portion3_start_rezero.py`, and `tests/test_final_rear_interface.py`: update the P3 diagnostic marker while preserving prior assertions.

### Task 1: Lock down reverse math and state contracts

**Files:**
- Create: `tests/test_portion3_direct_reverse.py`
- Test: `code/portion3_reverse_tracker.h`

- [ ] **Step 1: Write the failing host test**

Create a test that compiles a small C program including `portion3_reverse_tracker.h` and asserts:

```c
assert(portion3_reverse_motion_heading(10.0f) == -170.0f);
assert(portion3_reverse_initial_index(6) == 1);
assert(portion3_reverse_should_stop(0.10f, 4.0f, 5, 6));
assert(!portion3_reverse_should_stop(0.40f, 4.0f, 5, 6));
assert(portion3_reverse_safety_stop(10.2f, 10.0f, 0.15f));
```

The same Python test must assert that the Portion3 save block contains, in order:

```c
Flash_Store_Mode(route_setting_choice);
portion3_direct_reverse_prepare(p);
```

and does not contain `portion3_return_reset()`.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```powershell
python -m unittest tests.test_portion3_direct_reverse -v
```

Expected: FAIL because `code/portion3_reverse_tracker.h` and `portion3_direct_reverse_prepare()` do not exist.

- [ ] **Step 3: Add the minimal pure helper header**

Implement static inline helpers with no hardware dependencies:

```c
static inline float portion3_reverse_motion_heading(float yaw_deg);
static inline int portion3_reverse_initial_index(int route_length);
static inline float portion3_reverse_steering(float heading_error_deg,
        float target_distance_m, float wheel_base_m, float gain,
        float steering_limit_deg);
static inline int portion3_reverse_should_stop(float final_distance_m,
        float final_yaw_error_deg, int route_index, int route_length);
static inline int portion3_reverse_safety_stop(float travelled_m,
        float route_length_m, float overrun_m);
```

Use `atan2f`, `sinf`, angle normalization, a 0.12 m denominator floor, and explicit steering limiting. `portion3_reverse_steering()` returns the servo command with the reverse sign already applied.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run the same command. Expected: host C program passes; the integration source assertion still fails only because the save transition is not implemented yet.

### Task 2: Implement automatic save-to-reverse transition

**Files:**
- Modify: `code/guandao.c`
- Modify: `code/guandao.h`
- Test: `tests/test_portion3_direct_reverse.py`

- [ ] **Step 1: Extend the failing integration tests**

Assert that `portion3_direct_reverse_prepare()`:

```python
self.assertIn("portion3_direct_reverse_prepare(p);", save_body)
self.assertLess(save_body.index("Flash_Store_Mode"), save_body.index("portion3_direct_reverse_prepare"))
self.assertNotIn("portion3_return_reset();", save_body)
self.assertIn("main_mode = Guandao_portion_3;", prepare_body)
self.assertIn("conrtol_mode = DAOCHE;", prepare_body)
self.assertIn("daoche_flag = 1;", prepare_body)
self.assertNotIn("rear_motor_reset_odometry", prepare_body)
self.assertNotIn("IMU_yaw_rezero_start", prepare_body)
```

Also require an in-place swap of `recode_map[i]` and `recode_map[length - 1 - i]` after Flash storage, preserving coordinates without translation or rotation.

- [ ] **Step 2: Run and verify RED**

Run the focused unittest. Expected: FAIL on missing transition state and prepare function.

- [ ] **Step 3: Implement the minimal transition**

In `guandao.c`, add Portion3-only state fields for active/done state, route index, route length, start time, travelled distance, route distance, last pose, final target, steering command, and lookahead.

Implement:

```c
static uint8 portion3_direct_reverse_prepare(guandao_state *route);
static void portion3_direct_reverse_reset(void);
```

Preparation must validate at least three route points, calculate route length, reverse RAM points in place, preserve `route->current_state` and `Yaw_1`, set index to the first point after the saved endpoint, switch to `Guandao_portion_3`/`DAOCHE`, and never clear encoders or start IMU rezero.

In the save-pending block call preparation only after `Flash_Store_Mode(route_setting_choice)` succeeds synchronously. Invalid route preparation keeps zero outputs and starts braking.

- [ ] **Step 4: Run and verify GREEN**

Run the focused test. Expected: all transition assertions pass.

- [ ] **Step 5: Commit**

Stage only the new test, helper header, and `guandao` files, then commit:

```powershell
git commit -m "feat: start portion3 reverse after save"
```

### Task 3: Implement reverse tracking and protected stop

**Files:**
- Modify: `code/guandao.c`
- Test: `tests/test_portion3_direct_reverse.py`

- [ ] **Step 1: Write failing behavior contracts**

Require `guandao_trace()` to branch to the direct reverse update before the existing Portion3 rezero gate. Require the update function to:

```c
update_state(&portion_3, &guandao_ecd);
motion_heading = portion3_reverse_motion_heading(Yaw_1);
out_servo = portion3_reverse_steering(...);
daoche_flag = 1;
conrtol_mode = DAOCHE;
rear_motor_brake_start();
```

Assert that route progress only searches increasing indices in the already reversed RAM route, switches to fine reverse speed near the original start, and stops on either the final gate, route-distance overrun, or elapsed-time limit.

- [ ] **Step 2: Run and verify RED**

Run the focused unittest. Expected: FAIL because the update path does not exist.

- [ ] **Step 3: Implement the minimal state machine**

Add:

```c
static void portion3_direct_reverse_update(void);
```

The function updates pose, accumulates travelled distance, advances to the nearest forward index within a bounded window, selects a distance-based lookahead point, computes reverse steering, sets `daoche_speed` from the existing reverse speed planner, and starts braking when complete or unsafe. A completed state must continuously output zero and must not re-enter preparation.

- [ ] **Step 4: Run focused and related tests**

Run:

```powershell
python -m unittest tests.test_portion3_direct_reverse tests.test_portion3_save_consistency tests.test_portion3_start_rezero -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```powershell
git commit -m "feat: track portion3 route in reverse"
```

### Task 4: Add diagnostics and run full verification

**Files:**
- Modify: `code/guandao.h`
- Modify: `user/cpu0_main.c`
- Modify: `tests/test_portion3_save_consistency.py`
- Modify: `tests/test_portion3_start_rezero.py`
- Modify: `tests/test_final_rear_interface.py`
- Test: `tests/test_portion3_direct_reverse.py`

- [ ] **Step 1: Write failing diagnostic assertions**

Update the required marker to `P3AUTO,cfg=p3rev1` and require fields:

```text
p3Rev,revIdx,revD100,revCmd10
```

Require public read-only diagnostic accessors for active state, route index, final distance, and steering command.

- [ ] **Step 2: Run and verify RED**

Run the four P3-related test modules. Expected: FAIL on the old marker and missing fields.

- [ ] **Step 3: Implement diagnostics**

Expose diagnostic getters in `guandao.h`/`guandao.c` and add scaled values to the P3 serial format without changing control behavior.

- [ ] **Step 4: Verify encoding and source hygiene**

Run:

```powershell
python -c "from pathlib import Path; [p.read_text(encoding='utf-8') for p in [Path('code/portion3_reverse_tracker.h'), Path('code/guandao.c'), Path('code/guandao.h'), Path('user/cpu0_main.c'), Path('tests/test_portion3_direct_reverse.py')]]; print('UTF-8 OK')"
git diff --check
```

Expected: `UTF-8 OK`, no diff errors.

- [ ] **Step 5: Run the complete host suite**

Run:

```powershell
python -m unittest discover -s tests -v
```

Expected: all tests pass with zero failures/errors.

- [ ] **Step 6: Inspect final scope and commit**

Confirm `git diff --stat` contains only the planned files plus the two documentation files, and commit:

```powershell
git commit -m "test: verify portion3 direct reverse return"
```

- [ ] **Step 7: Record hardware validation boundary**

Report that TASKING/ADS Clean-Build-Link, flashing, and real-car confirmation remain required. Real-car acceptance is: save once, no Reset, no turn-around, automatic reverse motion, stable curve tracking, and brake stop at the original route start.