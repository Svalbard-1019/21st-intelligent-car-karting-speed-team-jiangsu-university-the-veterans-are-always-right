# Portion1 Adaptive Reverse Lookahead Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make portion1 taught-route reverse steering use a speed-adaptive metric lookahead while preserving low-speed parking precision.

**Architecture:** Extend the existing pure reverse speed-planning header with a deterministic lookahead function so the curve is independently testable. Use the selected cruise or fine reverse command to calculate the taught-route lookahead, expose that value through the existing reverse diagnostics, and leave the other reverse paths unchanged.

**Tech Stack:** C99 firmware, Python `unittest`, GCC host-side header tests, Git.

---

### Task 1: Define and test the adaptive lookahead curve

**Files:**
- Modify: `tests/test_portion1_menu_reverse_speed.py`
- Modify: `code/guandao_reverse_speed_planner.h`

- [ ] **Step 1: Write the failing test**

Add these assertions to the existing compiled C test:

```c
if(!close_enough(guandao_reverse_lookahead_m(-4.0f), 0.35f)) return 9;
if(!close_enough(guandao_reverse_lookahead_m(-10.0f), 0.71f)) return 10;
if(!close_enough(guandao_reverse_lookahead_m(-40.0f), 1.20f)) return 11;
if(!close_enough(guandao_reverse_lookahead_m(10.0f), 0.71f)) return 12;
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```powershell
python -m unittest tests.test_portion1_menu_reverse_speed.Portion1MenuReverseSpeedTests.test_menu_speed_builds_safe_cruise_and_fine_targets -v
```

Expected: compilation fails because `guandao_reverse_lookahead_m` is not defined.

- [ ] **Step 3: Implement the pure adaptive curve**

Add to `code/guandao_reverse_speed_planner.h`:

```c
static inline float guandao_reverse_lookahead_m(float speed_units)
{
    float speed_mps;
    float lookahead_m = 0.35f;

    if(speed_units < 0.0f) speed_units = -speed_units;
    speed_mps = speed_units * 0.1f;
    if(speed_mps > 0.4f)
    {
        lookahead_m += (speed_mps - 0.4f) * 0.60f;
    }
    if(lookahead_m > 1.20f) lookahead_m = 1.20f;
    return lookahead_m;
}
```

- [ ] **Step 4: Run the focused test and verify GREEN**

Run the command from Step 2. Expected: PASS.

### Task 2: Connect adaptive lookahead to taught reverse tracking

**Files:**
- Modify: `tests/test_portion1_menu_reverse_speed.py`
- Modify: `code/guandao.c`
- Modify: `code/guandao.h`

- [ ] **Step 1: Write the failing source integration test**

Assert that `guandao_taught_reverse_update()` calculates and uses `reverse_lookahead_m`, that `guandao_reverse_debug_lookahead()` exists, and that the old fixed `GUANDAO_TAUGHT_REVERSE_LOOKAHEAD` constant is absent.

- [ ] **Step 2: Run the integration test and verify RED**

Run:

```powershell
python -m unittest tests.test_portion1_menu_reverse_speed.Portion1MenuReverseSpeedTests.test_taught_reverse_uses_adaptive_metric_lookahead -v
```

Expected: FAIL because the adaptive integration is absent.

- [ ] **Step 3: Implement the taught-route integration**

In `guandao_taught_reverse_update()`:

```c
final_distance = get_distance(INS.current_state, portion1_taught_reverse_target);
if(final_distance <= GUANDAO_REVERSE_FINE_DIST
        || portion1_taught_reverse_index >= portion1_taught_reverse_length - 1)
{
    reverse_speed = reverse_plan.fine_units;
}
reverse_lookahead_m = guandao_reverse_lookahead_m(reverse_speed);
```

Use `reverse_lookahead_m` in the lookahead loop, store it in a resettable debug variable, and expose it through `guandao_reverse_debug_lookahead()`.

- [ ] **Step 4: Run the integration test and verify GREEN**

Run the command from Step 2. Expected: PASS.

### Task 3: Add serial evidence and firmware identification

**Files:**
- Modify: `tests/test_portion1_speed_planner.py`
- Modify: `user/cpu0_main.c`

- [ ] **Step 1: Write the failing serial-format test**

Require `cfg=p1spd6` and `revLd100=%ld` in the portion1 `AUTO` format.

- [ ] **Step 2: Run the serial test and verify RED**

Run:

```powershell
python -m unittest tests.test_portion1_speed_planner.Portion1SpeedPlannerTests.test_portion1_serial_line_exposes_speed_and_reverse_diagnostics -v
```

Expected: FAIL because the format still uses `cfg=p1spd5` and lacks `revLd100`.

- [ ] **Step 3: Update the serial format and argument list**

Change the marker to `cfg=p1spd6`, add `revLd100=%ld` after `revCmd10`, and pass:

```c
(long)Serial_Debug_Scale(guandao_reverse_debug_lookahead(), 100.0f),
```

- [ ] **Step 4: Run the serial test and verify GREEN**

Run the command from Step 2. Expected: PASS.

### Task 4: Verify, commit, and push

**Files:**
- Verify only the files listed in Tasks 1–3.

- [ ] **Step 1: Run the complete test suite**

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
```

Expected: all tests PASS.

- [ ] **Step 2: Check diffs and UTF-8 decoding**

Run `git diff --check` and decode every modified source/test file with Python using UTF-8. Expected: no diff errors and every decode succeeds.

- [ ] **Step 3: Stage only task files and commit**

```powershell
git add -- code/guandao.c code/guandao.h code/guandao_reverse_speed_planner.h user/cpu0_main.c tests/test_portion1_menu_reverse_speed.py tests/test_portion1_speed_planner.py
git commit -m "fix: adapt portion1 reverse lookahead to speed"
```

- [ ] **Step 4: Push and verify the remote**

```powershell
git push -u origin final
git ls-remote origin refs/heads/final
```

Expected: the remote `final` SHA equals local `HEAD`.
