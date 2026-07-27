# Portion1 Balance and PWM Release Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce fixed reverse offset, raise portion1 curve speeds, and make ordinary drive PWM release gradually without changing explicit stop or active braking.

**Architecture:** Keep speed-dependent reverse lookahead in the existing reverse planner, change only KMY curve ratios, and isolate ordinary-drive PWM slew in a small pure C helper. The normal PID path will clamp opposite-polarity requests and use KMY-specific release slew, while open-loop active braking keeps the existing symmetric PWM path.

**Tech Stack:** C99 firmware, Python `unittest`, host GCC tests, Git.

---

### Task 1: Rebalance reverse lookahead

**Files:**
- Modify: `tests/test_portion1_menu_reverse_speed.py`
- Modify: `code/guandao_reverse_speed_planner.h`

- [ ] Change the 1.0 m/s expected lookahead from `0.71f` to `0.59f`.
- [ ] Run the focused test and verify it fails against the current `0.60f` slope.
- [ ] Change the lookahead growth multiplier from `0.60f` to `0.40f`.
- [ ] Run the focused test and verify it passes, including the unchanged 1.20 m cap.

### Task 2: Raise portion1 curve ratios

**Files:**
- Modify: `tests/test_portion1_speed_planner.py`
- Modify: `code/guandao.c`

- [ ] Add failing expectations for KMY ratios `0.80/0.65/0.85/0.75/0.65` and unchanged hairpin `0.45`.
- [ ] Require the medium-speed curve cap to use `curve_speed_ratio` instead of a fixed `0.75f`.
- [ ] Run the source test and verify it fails on current values.
- [ ] Update only KMY constants and replace the fixed medium-speed cap with the route-specific ratio.
- [ ] Run the source test and verify KMS values remain unchanged.

### Task 3: Add ordinary-drive PWM release slew

**Files:**
- Create: `code/rear_motor/rear_pwm_slew.h`
- Create: `tests/test_rear_pwm_slew.py`
- Modify: `code/rear_motor/rear_motor.c`
- Modify: `code/rear_motor/rear_motor.h`

- [ ] Write a host C test for:
  - positive acceleration limited to 600;
  - positive PWM release limited to 300;
  - negative PWM magnitude release limited to 300;
  - opposite-polarity PID request clamped to zero for a nonzero target;
  - KMS-compatible 1000/1000 behavior.
- [ ] Run the test and verify compilation fails because the helper does not exist.
- [ ] Implement `rear_pwm_limit_request_sign()` and `rear_pwm_slew_step()` in the pure header.
- [ ] Add `pwm_release_limit` to the rear profile: KMY 300, KMS 1000.
- [ ] Route normal PID through sign clamping and release slew.
- [ ] Keep `rear_motor_open_loop_update()` and active braking on the existing symmetric slew path.
- [ ] Store and expose the sign-clamped PID request PWM.
- [ ] Run the focused helper and source tests.

### Task 4: Update diagnostics and firmware marker

**Files:**
- Modify: `tests/test_portion1_speed_planner.py`
- Modify: `user/cpu0_main.c`

- [ ] Require `cfg=p1spd7` and `pwmReq=%d` in the portion1 serial format.
- [ ] Run the serial source test and verify it fails.
- [ ] Add `rear_motor_get_requested_pwm()` to the format argument list.
- [ ] Run the serial source test and verify it passes.

### Task 5: Verify and publish

**Files:**
- Verify only files listed above.

- [ ] Run `python -m unittest discover -s tests -p "test_*.py" -v`.
- [ ] Run `git diff --check`.
- [ ] Decode every modified source and test file as UTF-8.
- [ ] Confirm `GUANDAO_P1_ACCEL_UNITS_PER_S` remains `35.0f`.
- [ ] Stage only task files and commit with `fix: rebalance portion1 curves and PWM release`.
- [ ] Fast-forward the feature branch into `final`.
- [ ] Run the full suite again on merged `final`.
- [ ] Push `final` and verify local/remote SHA equality.
