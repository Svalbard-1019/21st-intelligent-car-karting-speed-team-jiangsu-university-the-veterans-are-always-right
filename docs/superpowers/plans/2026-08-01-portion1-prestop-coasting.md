# Portion1 Pre-stop Coasting Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add speed-dependent Portion1 pre-stop coasting that reduces high-speed stopping distance without changing inertial tracking, steering, recorded route data, or reverse behavior.

**Architecture:** A standalone header-only policy computes the dynamic trigger distance and latches the pre-coast phase. `guandao.c` owns a read-only cached route-remaining calculation and exposes diagnostics, while `cpu0_main.c` alone selects either zero-PWM coasting or the existing speed PID. `rear_motor.c` gains a coast update that consumes speed samples but never clears odometry or applies reverse PWM.

**Tech Stack:** TC264 C99 firmware, Python `unittest`, host GCC tests, existing source-invariant tests.

---

## File map

- Create `code/portion1_precoast.h`: pure pre-coast policy and constants, with no hardware dependencies.
- Create `tests/test_portion1_precoast.py`: host-compiled policy tests plus source-level isolation and integration assertions.
- Modify `code/rear_motor/rear_motor.h`: declare the zero-PWM coast interface.
- Modify `code/rear_motor/rear_motor.c`: update speed feedback while forcing all rear PWM outputs to zero without resetting odometry.
- Modify `code/guandao.c`: cache route arc length, maintain Portion1 pre-coast state, and expose read-only diagnostics.
- Modify `code/guandao.h`: declare diagnostic/control getters used by the main-loop dispatcher and serial output.
- Modify `user/cpu0_main.c`: select coast versus capped 1.0 m/s PID and append diagnostics to the Portion1 serial record.

### Task 1: Pure pre-coast policy

**Files:**
- Create: `code/portion1_precoast.h`
- Create: `tests/test_portion1_precoast.py`

- [ ] **Step 1: Write the failing host-compiled policy test**

Create `tests/test_portion1_precoast.py` with a temporary C program that verifies the exact braking-distance formula, the `base_speed > 30` boundary, invalid-route rejection, latch persistence, zero-output threshold, and reset behavior:

```python
from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest

ROOT = Path(__file__).resolve().parents[1]

class Portion1PreCoastPolicyTests(unittest.TestCase):
    def test_policy_formula_thresholds_and_latch(self):
        source = textwrap.dedent(r"""
            #include <math.h>
            #include "portion1_precoast.h"

            static int near(float a, float b) { return fabsf(a - b) < 0.001f; }

            int main(void)
            {
                portion1_precoast_t state;
                portion1_precoast_reset(&state);
                if(!near(portion1_precoast_trigger_m(2.74f), 6.47f)) return 1;
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
        """)
        with tempfile.TemporaryDirectory() as temp_dir:
            src = Path(temp_dir) / "precoast_test.c"
            exe = Path(temp_dir) / "precoast_test.exe"
            src.write_text(source, encoding="utf-8")
            result = subprocess.run(
                ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                 "-I", str(ROOT / "code"), str(src), "-o", str(exe)],
                capture_output=True, text=True, encoding="utf-8", errors="replace")
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            run = subprocess.run([str(exe)], capture_output=True, text=True,
                                 encoding="utf-8", errors="replace")
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```powershell
python -m unittest tests.test_portion1_precoast.Portion1PreCoastPolicyTests.test_policy_formula_thresholds_and_latch -v
```

Expected: `FAIL` because `portion1_precoast.h` does not exist.

- [ ] **Step 3: Add the minimal standalone policy**

Create `code/portion1_precoast.h` with this public state and behavior:

```c
#ifndef CODE_PORTION1_PRECOAST_H_
#define CODE_PORTION1_PRECOAST_H_

#define PORTION1_PRECOAST_BASE_THRESHOLD 30.0f
#define PORTION1_PRECOAST_TARGET_MPS 1.0f
#define PORTION1_PRECOAST_RELEASE_MPS 1.15f
#define PORTION1_PRECOAST_DECEL_MPS2 0.60f
#define PORTION1_PRECOAST_RESPONSE_S 0.20f
#define PORTION1_PRECOAST_MARGIN_M 0.50f

typedef struct
{
    unsigned char latched;
    unsigned char coast_output;
    float remaining_m;
    float trigger_m;
} portion1_precoast_t;

static inline void portion1_precoast_reset(portion1_precoast_t *state)
{
    state->latched = 0u;
    state->coast_output = 0u;
    state->remaining_m = 0.0f;
    state->trigger_m = 0.0f;
}

static inline float portion1_precoast_trigger_m(float speed_mps)
{
    float braking = speed_mps * speed_mps
            - PORTION1_PRECOAST_TARGET_MPS * PORTION1_PRECOAST_TARGET_MPS;
    if(braking < 0.0f) braking = 0.0f;
    return braking / (2.0f * PORTION1_PRECOAST_DECEL_MPS2)
            + speed_mps * PORTION1_PRECOAST_RESPONSE_S
            + PORTION1_PRECOAST_MARGIN_M;
}

static inline void portion1_precoast_update(portion1_precoast_t *state,
        unsigned char route_valid, unsigned char forward_active,
        float base_speed_units, float actual_speed_mps, float remaining_m)
{
    if(!route_valid || !forward_active)
    {
        portion1_precoast_reset(state);
        return;
    }
    state->remaining_m = remaining_m;
    state->trigger_m = portion1_precoast_trigger_m(actual_speed_mps);
    if(!state->latched
            && base_speed_units > PORTION1_PRECOAST_BASE_THRESHOLD
            && actual_speed_mps > PORTION1_PRECOAST_RELEASE_MPS
            && remaining_m <= state->trigger_m)
    {
        state->latched = 1u;
    }
    state->coast_output = state->latched
            && actual_speed_mps > PORTION1_PRECOAST_RELEASE_MPS;
}

#endif
```

- [ ] **Step 4: Run the focused test and verify GREEN**

Run:

```powershell
python -m unittest tests.test_portion1_precoast.Portion1PreCoastPolicyTests.test_policy_formula_thresholds_and_latch -v
```

Expected: `OK` with 1 passing test.

- [ ] **Step 5: Commit the pure policy**

```powershell
git add -- code/portion1_precoast.h tests/test_portion1_precoast.py
git commit -m "feat: add portion1 pre-coast policy"
```

### Task 2: Rear motor zero-PWM coast interface

**Files:**
- Modify: `code/rear_motor/rear_motor.h`
- Modify: `code/rear_motor/rear_motor.c`
- Modify: `tests/test_portion1_precoast.py`

- [ ] **Step 1: Add a failing source-invariant test for feedback preservation**

Append this test to `Portion1PreCoastPolicyTests`:

```python
def test_rear_coast_updates_speed_without_resetting_odometry(self):
    source = (ROOT / "code/rear_motor/rear_motor.c").read_text(encoding="utf-8")
    header = (ROOT / "code/rear_motor/rear_motor.h").read_text(encoding="utf-8")
    self.assertIn("void rear_motor_coast_update(void);", header)
    body = source.split("void rear_motor_coast_update(void)", 1)[1]
    body = body.split("void rear_motor_open_loop_update", 1)[0]
    self.assertIn("rear_motor_take_speed_windows", body)
    self.assertIn("rear_motor_filter_speed", body)
    self.assertIn("rear_motor_set_pwm(0);", body)
    self.assertNotIn("rear_motor_stop", body)
    for forbidden in (
        "encoder_10ms = 0", "speed_window_build_pulses = 0",
        "speed_window_ready_pulses = 0", "odometry_total_pulses = 0",
        "rear_odometry_pose_buffer_init",
    ):
        self.assertNotIn(forbidden, body)
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```powershell
python -m unittest tests.test_portion1_precoast.Portion1PreCoastPolicyTests.test_rear_coast_updates_speed_without_resetting_odometry -v
```

Expected: `FAIL` because `rear_motor_coast_update()` is absent.

- [ ] **Step 3: Add the coast API and implementation**

Declare in `code/rear_motor/rear_motor.h`:

```c
void rear_motor_coast_update(void);
```

Implement immediately before `rear_motor_open_loop_update()` in `code/rear_motor/rear_motor.c`:

```c
void rear_motor_coast_update(void)
{
    int32 window_pulses;
    uint16 window_count;

    if(rear_motor_take_speed_windows(&window_pulses, &window_count))
    {
        float measured_pulses = (float)window_pulses / (float)window_count;
        encoder_100ms_last = (int32)measured_pulses;
        rear_motor_filter_speed(measured_pulses);
    }

    target_mps = 0.0f;
    requested_pwm = 0;
    integral = 0.0f;
    last_error = 0.0f;
    last_pwm = 0;
    rear_motor_set_pwm(0);
}
```

This deliberately calls the non-slew output helper so all four PWM channels become zero immediately, while the ISR-owned encoder windows and odometry buffer remain untouched.

- [ ] **Step 4: Run the focused tests and verify GREEN**

Run:

```powershell
python -m unittest tests.test_portion1_precoast -v
```

Expected: `OK` with 2 passing tests.

- [ ] **Step 5: Commit the motor interface**

```powershell
git add -- code/rear_motor/rear_motor.h code/rear_motor/rear_motor.c tests/test_portion1_precoast.py
git commit -m "feat: add odometry-safe rear coasting"
```

### Task 3: Portion1 route-distance integration and isolation guards

**Files:**
- Modify: `code/guandao.c`
- Modify: `code/guandao.h`
- Modify: `user/cpu0_main.c`
- Modify: `tests/test_portion1_precoast.py`

- [ ] **Step 1: Add failing integration and no-inertial-write tests**

Append these tests:

```python
def test_precoast_is_portion1_forward_only_and_caps_motor_target(self):
    guandao = (ROOT / "code/guandao.c").read_text(encoding="utf-8")
    main = (ROOT / "user/cpu0_main.c").read_text(encoding="utf-8")
    dispatch = main.split("static void Guandao_Rear_Motor_Update(void)", 1)[1]
    dispatch = dispatch.split("static int32 Serial_Debug_Scale", 1)[0]
    self.assertIn("main_mode == Guandao_portion_1", dispatch)
    self.assertIn("conrtol_mode == GUANDAO", dispatch)
    self.assertIn("guandao_portion1_precoast_output()", dispatch)
    self.assertIn("rear_motor_coast_update();", dispatch)
    self.assertIn("PORTION1_PRECOAST_TARGET_MPS", dispatch)
    self.assertLess(dispatch.index("rear_motor_brake_active()"),
                    dispatch.index("guandao_portion1_precoast_output()"))
    self.assertIn("portion1_precoast_update(&portion1_precoast", guandao)

def test_precoast_does_not_write_inertial_or_steering_state(self):
    source = (ROOT / "code/guandao.c").read_text(encoding="utf-8")
    block = source.split("static void guandao_portion1_precoast_update", 1)[1]
    block = block.split("void portion_1(void)", 1)[0]
    for forbidden in (
        "INS.current_state.x =", "INS.current_state.y =",
        "INS.current_state.theta =", "INS.current_point_index =",
        "out_servo =", "pursuit_contral_mode(", "update_state(",
    ):
        self.assertNotIn(forbidden, block)

def test_route_remaining_cache_is_built_once_and_read_only_during_tracking(self):
    source = (ROOT / "code/guandao.c").read_text(encoding="utf-8")
    prepare = source.split("static void guandao_portion1_precoast_prepare", 1)[1]
    prepare = prepare.split("static float guandao_portion1_remaining_m", 1)[0]
    remaining = source.split("static float guandao_portion1_remaining_m", 1)[1]
    remaining = remaining.split("static void guandao_portion1_precoast_update", 1)[0]
    self.assertIn("portion1_route_remaining_m[i]", prepare)
    self.assertIn("get_distance", prepare)
    self.assertNotIn("INS.current_point_index =", remaining)
    self.assertNotIn("INS.current_state", remaining.split("get_distance", 1)[1])
```

- [ ] **Step 2: Run the integration tests and verify RED**

Run:

```powershell
python -m unittest tests.test_portion1_precoast -v
```

Expected: the three new tests fail because the state, cache, getters, and dispatcher branch do not exist.

- [ ] **Step 3: Add cached route distance and state lifecycle**

In `code/guandao.c`, include `portion1_precoast.h`, add one `portion1_precoast_t`, one `float[MAX_LENGTH_INDEX]` cache, and helpers with these signatures:

```c
static portion1_precoast_t portion1_precoast;
static float portion1_route_remaining_m[MAX_LENGTH_INDEX];

static void guandao_portion1_precoast_prepare(void);
static float guandao_portion1_remaining_m(void);
static void guandao_portion1_precoast_update(uint8 reverse_route_valid);
```

`guandao_portion1_precoast_prepare()` must fill the cache backward from `daoche_point_length`, using `get_distance(INS.recode_map[i], INS.recode_map[i + 1])`. `guandao_portion1_remaining_m()` must clamp `INS.current_point_index` into `[0, daoche_point_length]` and return the cached tail plus only `get_distance(INS.current_state, INS.recode_map[index])`; it must not mutate the pose or index.

Use this exact policy call in the update helper:

```c
portion1_precoast_update(&portion1_precoast,
        reverse_route_valid,
        (portion1_reverse_state == 0u && daoche_flag == 0u),
        base_speed,
        rear_motor_get_speed_mps(),
        guandao_portion1_remaining_m());
```

Call `portion1_precoast_reset()` from `portion_1_reset()`, call `guandao_portion1_precoast_prepare()` once in the `portion1_state_flag == 0` initialization block, and call the update helper after `reverse_route_valid` is calculated in the normal forward branch. Reset it before entering the reverse wait state.

Expose these exact functions in both `code/guandao.c` and `code/guandao.h`:

```c
uint8 guandao_portion1_precoast_latched(void);
uint8 guandao_portion1_precoast_output(void);
float guandao_portion1_precoast_remaining_m(void);
float guandao_portion1_precoast_trigger_m(void);
void guandao_portion1_precoast_cancel(void);
```

- [ ] **Step 4: Gate only longitudinal motor output in the dispatcher**

In `Guandao_Rear_Motor_Update()`, keep active braking first. After computing the normal target and before the existing zero-target/PID branch, add:

```c
if(main_mode == Guandao_portion_1 && conrtol_mode == GUANDAO)
{
    if(guandao_portion1_precoast_output())
    {
        rear_motor_coast_update();
        return;
    }
    if(guandao_portion1_precoast_latched()
            && target_mps > PORTION1_PRECOAST_TARGET_MPS)
    {
        target_mps = PORTION1_PRECOAST_TARGET_MPS;
    }
}
else
{
    guandao_portion1_precoast_cancel();
}
```

Include `portion1_precoast.h` in `user/cpu0_main.c`. Do not assign `out_v_l`, `out_v_r`, `out_servo`, `INS.current_state`, or `INS.current_point_index` from the dispatcher.

- [ ] **Step 5: Run focused and existing braking tests**

Run:

```powershell
python -m unittest tests.test_portion1_precoast tests.test_subject_global_brake tests.test_rear_global_brake -v
```

Expected: all selected tests pass; explicit final braking still has priority over coasting.

- [ ] **Step 6: Commit the isolated integration**

```powershell
git add -- code/guandao.c code/guandao.h user/cpu0_main.c tests/test_portion1_precoast.py
git commit -m "feat: integrate portion1 pre-stop coasting"
```

### Task 4: Serial diagnostics and full regression

**Files:**
- Modify: `user/cpu0_main.c`
- Modify: `tests/test_portion1_precoast.py`

- [ ] **Step 1: Add a failing serial contract test**

Append:

```python
def test_serial_exposes_precoast_diagnostics(self):
    source = (ROOT / "user/cpu0_main.c").read_text(encoding="utf-8")
    start = source.index('"AUTO,')
    end = source.index("\\r\\n", start)
    auto_format = source[start:end]
    for field in ("preCoast=%u", "remain100=%ld", "coastTrig100=%ld", "coastOut=%u"):
        self.assertIn(field, auto_format)
    for getter in (
        "guandao_portion1_precoast_latched()",
        "guandao_portion1_precoast_remaining_m()",
        "guandao_portion1_precoast_trigger_m()",
        "guandao_portion1_precoast_output()",
    ):
        self.assertIn(getter, source)
```

- [ ] **Step 2: Run the serial test and verify RED**

Run:

```powershell
python -m unittest tests.test_portion1_precoast.Portion1PreCoastPolicyTests.test_serial_exposes_precoast_diagnostics -v
```

Expected: `FAIL` because the four fields are absent.

- [ ] **Step 3: Append the four fields without removing existing diagnostics**

Extend the Portion1 `AUTO` format string and argument list in matching order:

```c
",preCoast=%u,remain100=%ld,coastTrig100=%ld,coastOut=%u"
```

```c
(unsigned int)guandao_portion1_precoast_latched(),
(long)Serial_Debug_Scale(guandao_portion1_precoast_remaining_m(), 100.0f),
(long)Serial_Debug_Scale(guandao_portion1_precoast_trigger_m(), 100.0f),
(unsigned int)guandao_portion1_precoast_output()
```

Keep `revPlan`, `revFail`, raw IMU, kinematic yaw, PWM, and final brake fields unchanged.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run:

```powershell
python -m unittest tests.test_portion1_precoast -v
```

Expected: all pre-coast tests pass.

- [ ] **Step 5: Run full verification**

Run:

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
git diff --check
```

Expected: all tests pass and `git diff --check` prints no errors. Then validate every modified/new text file with strict UTF-8 decoding. If `cctc` remains unavailable, record that the TC264 firmware binary was not compiled locally.

- [ ] **Step 6: Commit diagnostics and tests**

```powershell
git add -- user/cpu0_main.c tests/test_portion1_precoast.py
git commit -m "test: expose portion1 pre-coast diagnostics"
```

## Real-car verification gate

Do not tune steering or inertial tracking during this gate. With the same recorded route, run speed settings 30, 35, and 40 three times each and verify:

```text
30: preCoast=0 and behavior matches the current baseline
35/40 while coastOut=1: pwm=0 and pwmReq=0, never negative
at app=1: act100 <= 120
throughout pre-coast: no stop-then-forward restart
at the entry line: lateral error degradation <= 10 cm versus baseline
after the normal wait: revSt advances directly to reverse and revPlan remains 0
```
