# Portion3 Adaptive Reverse Tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace Portion3 direct reverse's fixed five-point target with the same speed-adaptive metric lookahead used by Portion1, and reduce reverse speed before bends without touching inertial tracking or route-index progression.

**Architecture:** Extend the hardware-independent `portion3_reverse_tracker.h` with metric target selection and a hysteretic turn-speed planner. `guandao.c` caches reversed-route cumulative arc length during preparation, uses the pure helpers during direct reverse, and exposes diagnostics; `cpu0_main.c` only prints those diagnostics.

**Tech Stack:** TC264 C99 firmware, header-only control helpers, Python `unittest`, host GCC tests, source-invariant integration tests.

---

## File map

- Modify `code/portion3_reverse_tracker.h`: metric preview-index selection, hysteretic turn-level state, and signed reverse-speed limiting.
- Modify `tests/test_portion3_direct_reverse.py`: host math tests and integration/isolation contracts.
- Modify `code/guandao.c`: cumulative arc-length cache, adaptive target selection, turn-aware speed command, lifecycle reset, and diagnostic getters.
- Modify `code/guandao.h`: public diagnostic getter declarations.
- Modify `user/cpu0_main.c`: append `revLd100`, `revTurn`, and `revSpd10` to `P3AUTO`.
- Create `code/remote_one_shot.h`: hardware-independent press-edge latch used by CH5.
- Create `tests/test_portion3_ch5_trigger.py`: host latch test plus SBUS, save-flow, and serial contracts.
- Modify `code/RemteControl.c`: decode SBUS CH5 into `x6f_out[4]` with receiver-loss reset.

### Task 1: Pure metric lookahead and curve-speed policy

**Files:**
- Modify: `tests/test_portion3_direct_reverse.py:30-90`
- Modify: `code/portion3_reverse_tracker.h`

- [ ] **Step 1: Change the host test to specify metric preview and hysteresis**

In `test_reverse_tracker_math_on_host`, declare state and non-uniform cumulative route distances:

```c
float route_m[5] = {0.0f, 0.15f, 0.55f, 0.75f, 1.40f};
portion3_reverse_turn_state_t turn_state;
```

Replace the fixed-point preview assertions with:

```c
assert(portion3_reverse_metric_preview_index(route_m, 1, 5, 0.50f) == 3);
assert(portion3_reverse_metric_preview_index(route_m, 3, 5, 1.20f) == 4);
assert(portion3_reverse_metric_preview_index(route_m, -2, 5, 0.10f) == 1);
```

Append exact turn-level, hysteresis, signed-speed, and fine-speed assertions:

```c
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
```

- [ ] **Step 2: Run the host test and verify RED**

Run:

```powershell
python -m unittest tests.test_portion3_direct_reverse.Portion3DirectReverseMathTests.test_reverse_tracker_math_on_host -v
```

Expected: `FAIL` because `portion3_reverse_turn_state_t`, `portion3_reverse_metric_preview_index`, `portion3_reverse_turn_reset`, `portion3_reverse_turn_level`, and `portion3_reverse_curve_speed` do not exist.

- [ ] **Step 3: Implement metric preview selection**

Add to `code/portion3_reverse_tracker.h`:

```c
static inline int portion3_reverse_metric_preview_index(
        const float *route_m, int route_index, int route_length,
        float lookahead_m)
{
    int preview_index;
    float target_m;

    if(route_length <= 0) return 0;
    if(route_index < 0) route_index = 0;
    if(route_index >= route_length) route_index = route_length - 1;
    if(lookahead_m < 0.0f) lookahead_m = 0.0f;
    target_m = route_m[route_index] + lookahead_m;
    preview_index = route_index;
    while(preview_index < route_length - 1
            && route_m[preview_index] < target_m)
    {
        preview_index++;
    }
    return preview_index;
}
```

- [ ] **Step 4: Implement hysteretic turn level and signed speed limiting**

Add to the same header:

```c
typedef struct
{
    unsigned char level;
} portion3_reverse_turn_state_t;

static inline void portion3_reverse_turn_reset(
        portion3_reverse_turn_state_t *state)
{
    state->level = 0u;
}

static inline unsigned char portion3_reverse_turn_level(
        portion3_reverse_turn_state_t *state, float heading_error_deg)
{
    float magnitude = fabsf(heading_error_deg);
    unsigned char level = state->level;

    if(level == 0u)
    {
        if(magnitude >= 45.0f) level = 3u;
        else if(magnitude >= 30.0f) level = 2u;
        else if(magnitude >= 15.0f) level = 1u;
    }
    else if(level == 1u)
    {
        if(magnitude >= 45.0f) level = 3u;
        else if(magnitude >= 30.0f) level = 2u;
        else if(magnitude < 10.0f) level = 0u;
    }
    else if(level == 2u)
    {
        if(magnitude >= 45.0f) level = 3u;
        else if(magnitude < 25.0f) level = (magnitude >= 15.0f) ? 1u : 0u;
    }
    else if(magnitude < 40.0f)
    {
        if(magnitude >= 30.0f) level = 2u;
        else if(magnitude >= 15.0f) level = 1u;
        else level = 0u;
    }

    state->level = level;
    return level;
}

static inline float portion3_reverse_curve_speed(
        float cruise_speed, float fine_speed,
        unsigned char turn_level, unsigned char fine_active)
{
    float ratio = 1.0f;
    float command;

    if(turn_level >= 3u) ratio = 0.55f;
    else if(turn_level == 2u) ratio = 0.70f;
    else if(turn_level == 1u) ratio = 0.85f;
    command = cruise_speed * ratio;
    if(fine_active && fabsf(fine_speed) < fabsf(command)) command = fine_speed;
    return command;
}
```

- [ ] **Step 5: Run the host test and verify GREEN**

Run:

```powershell
python -m unittest tests.test_portion3_direct_reverse.Portion3DirectReverseMathTests.test_reverse_tracker_math_on_host -v
```

Expected: `OK` with one passing test and no compiler warnings.

- [ ] **Step 6: Commit the pure policy**

```powershell
git add -- code/portion3_reverse_tracker.h tests/test_portion3_direct_reverse.py
git commit -m "feat: add adaptive portion3 reverse policy"
```

### Task 2: Integrate cached arc length without changing inertial tracking

**Files:**
- Modify: `tests/test_portion3_direct_reverse.py:92-227`
- Modify: `code/guandao.c:90-545`
- Modify: `code/guandao.h:339-343`

- [ ] **Step 1: Replace the obsolete fixed-preview integration test**

Replace `test_reverse_uses_portion1_forward_five_point_preview` with:

```python
def test_reverse_uses_portion1_metric_lookahead_and_curve_speed(self):
    signature = "static void portion3_direct_reverse_update(void)"
    update_body = braced_function_body(self.guandao_c, signature)
    self.assertNotIn("PORTION3_DIRECT_REVERSE_PREVIEW_STEPS", self.guandao_c)
    self.assertIn("guandao_reverse_lookahead_m(reference_speed)", update_body)
    self.assertIn("portion3_reverse_metric_preview_index(", update_body)
    self.assertIn("portion3_reverse_turn_level(", update_body)
    self.assertIn("portion3_reverse_curve_speed(", update_body)
```

Add cache and isolation assertions:

```python
def test_reverse_prepare_builds_arc_cache_after_ram_route_reversal(self):
    signature = "static uint8 portion3_direct_reverse_prepare(guandao_state *route)"
    body = braced_function_body(self.guandao_c, signature)
    reverse = body.index("route->recode_map[i] = route->recode_map[length - 1 - i];")
    cache = body.index("portion3_direct_reverse_route_m[i]")
    self.assertLess(reverse, cache)
    self.assertIn("get_distance(route->recode_map[i - 1], route->recode_map[i])", body)

def test_adaptive_target_does_not_replace_pose_or_index_updates(self):
    signature = "static void portion3_direct_reverse_update(void)"
    body = braced_function_body(self.guandao_c, signature)
    self.assertEqual(body.count("update_state(&portion_3, &guandao_ecd);"), 1)
    self.assertEqual(
        body.count("portion_3.current_point_index = portion3_direct_reverse_route_index;"),
        1,
    )
    target_call = body.split("portion3_reverse_metric_preview_index(", 1)[1]
    target_call = target_call.split(");", 1)[0]
    self.assertNotIn("current_state =", target_call)
    self.assertNotIn("current_point_index =", target_call)
```

- [ ] **Step 2: Run integration tests and verify RED**

Run:

```powershell
python -m unittest tests.test_portion3_direct_reverse.Portion3DirectReverseIntegrationTests -v
```

Expected: the new tests fail because direct reverse still uses the fixed five-point preview and has no arc-length cache or curve-speed planner.

- [ ] **Step 3: Add direct-reverse cache and debug state**

In `code/guandao.c`, remove `PORTION3_DIRECT_REVERSE_PREVIEW_STEPS` and add module state beside the existing direct-reverse globals:

```c
static float portion3_direct_reverse_route_m[MAX_LENGTH_INDEX];
static portion3_reverse_turn_state_t portion3_direct_reverse_turn_state;
static float portion3_direct_reverse_lookahead_m = 0.0f;
static uint8 portion3_direct_reverse_turn_level = 0u;
static float portion3_direct_reverse_speed_command = 0.0f;
```

Reset all new runtime values in `portion3_direct_reverse_reset()`:

```c
portion3_reverse_turn_reset(&portion3_direct_reverse_turn_state);
portion3_direct_reverse_lookahead_m = 0.0f;
portion3_direct_reverse_turn_level = 0u;
portion3_direct_reverse_speed_command = 0.0f;
```

After the existing RAM route reversal loop in `portion3_direct_reverse_prepare()`, build the cumulative cache:

```c
portion3_direct_reverse_route_m[0] = 0.0f;
for(int16 i = 1; i < length; i++)
{
    portion3_direct_reverse_route_m[i] = portion3_direct_reverse_route_m[i - 1]
            + get_distance(route->recode_map[i - 1], route->recode_map[i]);
}
```

- [ ] **Step 4: Replace fixed target selection and add curve-speed output**

In `portion3_direct_reverse_update()`, add locals:

```c
float reference_speed;
uint8 fine_active;
```

Replace the old cruise/fine and fixed-preview block with:

```c
fine_active = (portion3_direct_reverse_final_dist <= PORTION3_DIRECT_REVERSE_FINE_DIST
        || portion3_direct_reverse_route_index >= length - 2);
reference_speed = fine_active ? reverse_plan.fine_units : reverse_plan.cruise_units;
portion3_direct_reverse_lookahead_m = guandao_reverse_lookahead_m(reference_speed);
lookahead_index = portion3_reverse_metric_preview_index(
        portion3_direct_reverse_route_m,
        portion3_direct_reverse_route_index,
        length,
        portion3_direct_reverse_lookahead_m);
target = portion_3.recode_map[lookahead_index];
```

Immediately after calculating `heading_error`, compute speed:

```c
portion3_direct_reverse_turn_level = portion3_reverse_turn_level(
        &portion3_direct_reverse_turn_state, heading_error);
reverse_speed = portion3_reverse_curve_speed(
        reverse_plan.cruise_units,
        reverse_plan.fine_units,
        portion3_direct_reverse_turn_level,
        fine_active);
portion3_direct_reverse_speed_command = reverse_speed;
```

Keep the existing steering calculation, steering gain, steering limit, rate limiter, index progression, terminal crossing, and brake calls unchanged.

- [ ] **Step 5: Expose diagnostic getters**

Add implementations beside the existing Portion3 reverse getters and declarations in `code/guandao.h`:

```c
float guandao_portion3_reverse_lookahead(void)
{
    return portion3_direct_reverse_lookahead_m;
}

uint8 guandao_portion3_reverse_turn_level(void)
{
    return portion3_direct_reverse_turn_level;
}

float guandao_portion3_reverse_speed_command(void)
{
    return portion3_direct_reverse_speed_command;
}
```

```c
float guandao_portion3_reverse_lookahead(void);
uint8 guandao_portion3_reverse_turn_level(void);
float guandao_portion3_reverse_speed_command(void);
```

- [ ] **Step 6: Run focused Portion3 tests and verify GREEN**

Run:

```powershell
python -m unittest tests.test_portion3_direct_reverse -v
```

Expected: all Portion3 direct reverse tests pass.

- [ ] **Step 7: Commit the integration**

```powershell
git add -- code/guandao.c code/guandao.h tests/test_portion3_direct_reverse.py
git commit -m "fix: stabilize fast portion3 reverse turns"
```

### Task 3: Serial contract and complete regression

**Files:**
- Modify: `tests/test_portion3_direct_reverse.py:208-225`
- Modify: `user/cpu0_main.c:465-520`

- [ ] **Step 1: Add failing diagnostics contract assertions**

Extend `test_diagnostics_identify_direct_reverse_firmware`:

```python
for field in ("revLd100=%ld", "revTurn=%u", "revSpd10=%ld"):
    self.assertIn(field, self.main_c)
for declaration in (
    "float guandao_portion3_reverse_lookahead(void);",
    "uint8 guandao_portion3_reverse_turn_level(void);",
    "float guandao_portion3_reverse_speed_command(void);",
):
    self.assertIn(declaration, self.guandao_h)
```

- [ ] **Step 2: Run the diagnostics test and verify RED**

Run:

```powershell
python -m unittest tests.test_portion3_direct_reverse.Portion3DirectReverseIntegrationTests.test_diagnostics_identify_direct_reverse_firmware -v
```

Expected: `FAIL` because the three `P3AUTO` fields are absent.

- [ ] **Step 3: Append diagnostic fields and matching arguments**

Insert after `revCmd10` in the `P3AUTO` format string:

```c
,revLd100=%ld,revTurn=%u,revSpd10=%ld
```

Insert the matching arguments after `guandao_portion3_reverse_steer_command()`:

```c
(long)Serial_Debug_Scale(
        guandao_portion3_reverse_lookahead(), 100.0f),
(unsigned int)guandao_portion3_reverse_turn_level(),
(long)Serial_Debug_Scale(
        guandao_portion3_reverse_speed_command(), 10.0f),
```

Do not remove or reorder the existing IMU, GPS, pose, steering, motor, encoder, or brake fields.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run:

```powershell
python -m unittest tests.test_portion3_direct_reverse -v
```

Expected: all Portion3 direct reverse tests pass.

- [ ] **Step 5: Run full verification**

Run:

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
git diff --check
```

Expected: all tests pass and `git diff --check` reports no errors. Strictly decode `code/portion3_reverse_tracker.h`, `code/guandao.c`, `code/guandao.h`, `user/cpu0_main.c`, and `tests/test_portion3_direct_reverse.py` as UTF-8. If `cctc` is unavailable, record that the TC264 firmware binary could not be compiled locally.

- [ ] **Step 6: Commit diagnostics and tests**

```powershell
git add -- user/cpu0_main.c tests/test_portion3_direct_reverse.py
git commit -m "test: expose portion3 adaptive reverse diagnostics"
```

### Task 4: CH5 one-press save and direct reverse

**Files:**
- Create: `code/remote_one_shot.h`
- Create: `tests/test_portion3_ch5_trigger.py`
- Modify: `code/RemteControl.c:130-173`
- Modify: `code/guandao.c:2700-2840`
- Modify: `code/guandao.h:339-370`
- Modify: `user/cpu0_main.c:288-340`

- [ ] **Step 1: Write a failing host test for one-shot press behavior**

Create `tests/test_portion3_ch5_trigger.py` with:

```python
from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest

ROOT = Path(__file__).resolve().parents[1]

class Portion3Ch5TriggerTests(unittest.TestCase):
    def test_one_shot_fires_once_until_release(self):
        source = textwrap.dedent(r"""
            #include "remote_one_shot.h"
            int main(void)
            {
                remote_one_shot_t trigger;
                remote_one_shot_reset(&trigger);
                if(remote_one_shot_update(&trigger, 0u)) return 1;
                if(!remote_one_shot_update(&trigger, 1u)) return 2;
                if(remote_one_shot_update(&trigger, 1u)) return 3;
                if(remote_one_shot_update(&trigger, 1u)) return 4;
                if(remote_one_shot_update(&trigger, 0u)) return 5;
                if(!remote_one_shot_update(&trigger, 1u)) return 6;
                return 0;
            }
        """)
        with tempfile.TemporaryDirectory() as temp_dir:
            src = Path(temp_dir) / "remote_one_shot_test.c"
            exe = Path(temp_dir) / "remote_one_shot_test.exe"
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

- [ ] **Step 2: Run the latch test and verify RED**

Run:

```powershell
python -m unittest tests.test_portion3_ch5_trigger.Portion3Ch5TriggerTests.test_one_shot_fires_once_until_release -v
```

Expected: `FAIL` because `remote_one_shot.h` does not exist.

- [ ] **Step 3: Implement the standalone latch**

Create `code/remote_one_shot.h`:

```c
#ifndef CODE_REMOTE_ONE_SHOT_H_
#define CODE_REMOTE_ONE_SHOT_H_

typedef struct
{
    unsigned char armed;
} remote_one_shot_t;

static inline void remote_one_shot_reset(remote_one_shot_t *state)
{
    state->armed = 1u;
}

static inline unsigned char remote_one_shot_update(
        remote_one_shot_t *state, unsigned char pressed)
{
    if(!pressed)
    {
        state->armed = 1u;
        return 0u;
    }
    if(!state->armed) return 0u;
    state->armed = 0u;
    return 1u;
}

#endif
```

- [ ] **Step 4: Run the latch test and verify GREEN**

Run:

```powershell
python -m unittest tests.test_portion3_ch5_trigger.Portion3Ch5TriggerTests.test_one_shot_fires_once_until_release -v
```

Expected: `OK` with one passing test.

- [ ] **Step 5: Add failing SBUS and save-flow contracts**

Append:

```python
def test_sbus_decodes_ch5_and_clears_it_on_receiver_loss(self):
    source = (ROOT / "code/RemteControl.c").read_text(encoding="utf-8")
    body = source.split("void sbus_rc_control(void)", 1)[1].split(
        "void hotRc_Show", 1
    )[0]
    self.assertIn("ch_reverse = uart_receiver.channel[4];", body)
    self.assertIn("x6f_out[4] = (ch_reverse > 1500u) ? 200 : 100;", body)
    receiver_lost = body.split("if(uart_receiver.state == 0)", 1)[1].split(
        "return;", 1
    )[0]
    self.assertIn("x6f_out[4] = 100;", receiver_lost)

def test_ch5_reuses_existing_portion3_pending_save_flow(self):
    source = (ROOT / "code/guandao.c").read_text(encoding="utf-8")
    body = source.split("void guandao_recode(guandao_state * state)", 1)[1]
    body = body.split("void guandao_record_session_reset", 1)[0]
    self.assertIn("remote_one_shot_update(&portion3_ch5_trigger", body)
    self.assertIn("x6f_out[4] == 200", body)
    self.assertIn("route_setting_choice == 2", body)
    self.assertIn("p == &portion_3", body)
    self.assertIn("p->length_index > 1", body)
    trigger = body.index("portion3_save_pending = 1;")
    pending = body.index("if(portion3_save_pending)")
    self.assertLess(trigger, pending)
    self.assertIn("rear_motor_brake_start();", body[trigger:pending])
    self.assertNotIn("portion3_direct_reverse_prepare", body[trigger:pending])

def test_record_serial_exposes_ch5_trigger(self):
    source = (ROOT / "user/cpu0_main.c").read_text(encoding="utf-8")
    start = source.index('"REC,')
    end = source.index("\\r\\n", start)
    record_format = source[start:end]
    self.assertIn("ch5=%d", record_format)
    self.assertIn("p3Trig=%u", record_format)
    self.assertIn("x6f_out[4]", source)
    self.assertIn("guandao_portion3_ch5_triggered()", source)
```

- [ ] **Step 6: Run the contracts and verify RED**

Run:

```powershell
python -m unittest tests.test_portion3_ch5_trigger -v
```

Expected: three new tests fail because CH5 is not decoded or connected to the pending-save flow and diagnostics.

- [ ] **Step 7: Decode CH5 with receiver-loss protection**

In `sbus_rc_control()` declare `uint16 ch_reverse;`, set `x6f_out[4] = 100;` in the receiver-loss branch, then add:

```c
ch_reverse = uart_receiver.channel[4];
x6f_out[4] = (ch_reverse > 1500u) ? 200 : 100;
```

Keep CH1 steering, CH2 throttle, CH3 save, and CH4 stop mappings unchanged.

- [ ] **Step 8: Connect the one-shot to the existing Portion3 pending save**

Include `remote_one_shot.h` in `code/guandao.c` and add:

```c
static remote_one_shot_t portion3_ch5_trigger = {1u};
static uint8 portion3_ch5_triggered = 0u;
```

After `update_state(p, &guandao_ecd);` and before `if(portion3_save_pending)`, add:

```c
if(remote_one_shot_update(&portion3_ch5_trigger,
        (uint8)(x6f_out[4] == 200)))
{
    if(route_setting_choice == 2 && p == &portion_3
            && p->length_index > 1
            && !portion3_save_pending && !guandao_record_saved)
    {
        portion3_ch5_triggered = 1u;
        portion3_save_pending = 1;
        rear_motor_brake_start();
    }
}
```

Keep `portion3_ch5_triggered` latched after a successful press so the periodic serial output cannot miss it. Reset the one-shot state and diagnostic only when a new recording session starts. Expose:

```c
uint8 guandao_portion3_ch5_triggered(void)
{
    return portion3_ch5_triggered;
}
```

and declare `uint8 guandao_portion3_ch5_triggered(void);` in `code/guandao.h`.

- [ ] **Step 9: Append record diagnostics**

Add `ch5=%d,p3Trig=%u` to the existing `REC` format and matching arguments:

```c
x6f_out[4],
(unsigned int)guandao_portion3_ch5_triggered(),
```

- [ ] **Step 10: Run focused and full verification**

Run:

```powershell
python -m unittest tests.test_portion3_ch5_trigger tests.test_portion3_direct_reverse tests.test_portion3_save_consistency -v
python -m unittest discover -s tests -p "test_*.py" -v
git diff --check
```

Expected: all focused and full tests pass; strict UTF-8 decoding succeeds for every modified/new text file. If `cctc` remains unavailable, record that the TC264 firmware binary was not compiled locally.

- [ ] **Step 11: Commit CH5 support**

```powershell
git add -- code/remote_one_shot.h code/RemteControl.c code/guandao.c code/guandao.h user/cpu0_main.c tests/test_portion3_ch5_trigger.py
git commit -m "feat: trigger portion3 return with ch5"
```

## Real-car verification gate

Use one unchanged Portion3 recording and run reverse settings -10, -15, and -20 three times each:

```text
Higher speed: revLd100 is larger than at lower speed
revTurn 0/1/2/3: revSpd10 follows 100%/85%/70%/55% of cruise unless fine speed is lower
Late route: revCmd10 does not remain saturated near +/-320
Tracking: no repeated left-right correction and reverse index remains monotonic
Finish: existing p3Stop cause and final brake still occur normally
Isolation: IMU/pose fields remain continuous and Portion1 behavior is unchanged
CH5: one press brakes, saves the stopped endpoint, then enters direct reverse
CH5 held: no duplicate save; release and press again rearms the trigger
```
