# Portion3 Shared Speed Planner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Portion3 use the exact Portion1 straight, curve, acceleration, deceleration, and final-approach speed behavior through one continuous shared planner state.

**Architecture:** Keep the existing inline `guandao_speed_planner_t` implementation and use one statically initialized planner for route choices 0 and 2. Route-specific geometry, startup rezero, endpoint detection, and steering preview remain separate; only the speed-request classification and rate-limiting path becomes shared.

**Tech Stack:** Infineon AURIX C firmware, Python `unittest` source-contract tests, host GCC helper test.

---

### Task 1: Add failing shared-speed behavior tests

**Files:**
- Create: `tests/test_portion3_shared_speed_planner.py`
- Modify: `tests/test_final_integration.py:47-53`
- Test: `tests/test_portion3_shared_speed_planner.py`

- [ ] **Step 1: Write the failing shared-planner source tests**

Create `tests/test_portion3_shared_speed_planner.py`:

```python
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def function_body(source: str, start_marker: str, end_marker: str) -> str:
    start = source.index(start_marker)
    end = source.index(end_marker, start)
    return source[start:end]


class Portion3SharedSpeedPlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (ROOT / "code" / "guandao.c").read_text(encoding="utf-8")
        cls.pursuit = function_body(
            cls.source,
            "void pursuit_contral_mode(",
            "float speed_calculate(",
        )
        cls.portion1_reset = function_body(
            cls.source,
            "void portion_1_reset(void)",
            "void portion_1(void)",
        )
        cls.portion3_reset = function_body(
            cls.source,
            "void portion3_return_reset(void)",
            "void Guandao_Points_Show(",
        )

    def test_portion1_and_portion3_share_one_continuous_planner(self):
        self.assertIn(
            "static guandao_speed_planner_t portion1_speed_planner = {MIN_SPEED, 0u};",
            self.source,
        )
        self.assertNotIn(
            "guandao_speed_planner_reset(&portion1_speed_planner",
            self.portion1_reset,
        )
        self.assertNotIn(
            "guandao_speed_planner_reset(&portion1_speed_planner",
            self.portion3_reset,
        )
        self.assertIn(
            "if(route_setting_choice == 0 || route_setting_choice == 2)",
            self.pursuit,
        )

    def test_portion3_uses_portion1_turn_window_and_hysteresis(self):
        self.assertIn(
            "(route_setting_choice == 0 || route_setting_choice == 2)",
            self.pursuit,
        )
        self.assertIn("GUANDAO_P1_TURN_WINDOW_M", self.pursuit)
        self.assertIn(
            "guandao_speed_turn_level(\n                &portion1_speed_planner, upcoming_turn)",
            self.pursuit,
        )

    def test_portion3_uses_portion1_curve_ratios(self):
        self.assertNotIn("GUANDAO_KMS_CURVE_SPEED_RATIO", self.source)
        self.assertNotIn("GUANDAO_KMS_SHARP_TURN_SPEED_RATIO", self.source)
        self.assertNotIn("GUANDAO_KMS_ACCUM_TURN_", self.source)
        self.assertIn(
            "float curve_speed_ratio = GUANDAO_KMY_CURVE_SPEED_RATIO;",
            self.pursuit,
        )
        self.assertIn(
            "float sharp_turn_speed_ratio = GUANDAO_KMY_SHARP_TURN_SPEED_RATIO;",
            self.pursuit,
        )
        self.assertIn(
            "float accum_turn_slow_ratio = GUANDAO_KMY_ACCUM_TURN_SLOW_RATIO;",
            self.pursuit,
        )
        self.assertIn(
            "float accum_turn_medium_ratio = GUANDAO_KMY_ACCUM_TURN_MEDIUM_RATIO;",
            self.pursuit,
        )
        self.assertIn(
            "float accum_turn_sharp_ratio = GUANDAO_KMY_ACCUM_TURN_SHARP_RATIO;",
            self.pursuit,
        )

    def test_portion3_uses_shared_rate_limiter_without_entry_reset(self):
        self.assertIn(
            "if(route_setting_choice == 0 || route_setting_choice == 2)",
            self.pursuit,
        )
        self.assertIn(
            "guandao_speed_rate_limit(\n"
            "                   &portion1_speed_planner, v_center,",
            self.pursuit,
        )
        self.assertNotIn(
            "guandao_speed_planner_reset(&portion1_speed_planner",
            self.source,
        )


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Update the existing integration contract to the approved ratios**

Replace the KMS-specific assertions in `test_subject_three_has_latest_speed_and_finish_rules` with:

```python
        self.assertIn("GUANDAO_KMY_CURVE_SPEED_RATIO      0.80f", self.guandao_c)
        self.assertIn("GUANDAO_KMY_SHARP_TURN_SPEED_RATIO 0.65f", self.guandao_c)
        self.assertNotIn("GUANDAO_KMS_CURVE_SPEED_RATIO", self.guandao_c)
        self.assertNotIn("GUANDAO_KMS_SHARP_TURN_SPEED_RATIO", self.guandao_c)
        self.assertNotIn("GUANDAO_KMS_ACCUM_TURN_", self.guandao_c)
```

- [ ] **Step 3: Run the focused tests and verify RED**

Run:

```powershell
python -m unittest tests.test_portion3_shared_speed_planner tests.test_final_integration -v
```

Expected: the new shared-planner tests fail because Portion3 still selects KMS ratios, uses a point-count turn window, skips the rate limiter, and Portion1 still resets the planner.

- [ ] **Step 4: Commit the failing tests**

```powershell
git add -- tests/test_portion3_shared_speed_planner.py tests/test_final_integration.py
git commit -m "test: cover shared portion speed planner"
```

### Task 2: Share Portion1 speed behavior with Portion3

**Files:**
- Modify: `code/guandao.c:135-195`
- Modify: `code/guandao.c:1237-1352`
- Modify: `code/guandao.c:1849-2223`
- Test: `tests/test_portion3_shared_speed_planner.py`
- Test: `tests/test_portion1_speed_planner.py`
- Test: `tests/test_final_integration.py`

- [ ] **Step 1: Statically initialize the one shared planner**

Replace:

```c
static guandao_speed_planner_t portion1_speed_planner;
```

with:

```c
static guandao_speed_planner_t portion1_speed_planner = {MIN_SPEED, 0u};
```

Keep `portion1_speed_last_ms` as the one shared rate-limit timestamp.

- [ ] **Step 2: Remove the obsolete Portion3-only low-speed ratios**

Delete:

```c
#define GUANDAO_KMS_CURVE_SPEED_RATIO      0.70f
#define GUANDAO_KMS_SHARP_TURN_SPEED_RATIO 0.55f
#define GUANDAO_KMS_ACCUM_TURN_SLOW_RATIO  0.80f
#define GUANDAO_KMS_ACCUM_TURN_MEDIUM_RATIO 0.70f
#define GUANDAO_KMS_ACCUM_TURN_SHARP_RATIO 0.70f
```

Keep the Portion1/KMY constants unchanged.

- [ ] **Step 3: Preserve shared speed state across both subject entries**

Remove these calls from `portion_1_reset()` and the Portion1 start transition:

```c
guandao_speed_planner_reset(&portion1_speed_planner, MIN_SPEED);
portion1_speed_last_ms = 0u;
```

and:

```c
guandao_speed_planner_reset(&portion1_speed_planner, MIN_SPEED);
portion1_speed_last_ms = system_getval_ms();
```

Do not add either call to `portion3_return_reset()`.

- [ ] **Step 4: Select Portion1 ratios for both routes**

Replace the five route-dependent ratio expressions at the start of `pursuit_contral_mode()` with:

```c
    float curve_speed_ratio = GUANDAO_KMY_CURVE_SPEED_RATIO;
    float sharp_turn_speed_ratio = GUANDAO_KMY_SHARP_TURN_SPEED_RATIO;
    float accum_turn_slow_ratio = GUANDAO_KMY_ACCUM_TURN_SLOW_RATIO;
    float accum_turn_medium_ratio = GUANDAO_KMY_ACCUM_TURN_MEDIUM_RATIO;
    float accum_turn_sharp_ratio = GUANDAO_KMY_ACCUM_TURN_SHARP_RATIO;
```

- [ ] **Step 5: Use the distance turn window and hysteresis for both routes**

Replace the turn-window selection and level condition with:

```c
    upcoming_turn = (route_setting_choice == 0 || route_setting_choice == 2)
            ? guandao_portion1_distance_turn(
                    state, state->current_point_index, GUANDAO_P1_TURN_WINDOW_M)
            : guandao_accumulated_route_turn(state, state->current_point_index, 12);
    max_single_turn = guandao_max_route_turn(state, state->current_point_index, 12);
    if(route_setting_choice == 0 || route_setting_choice == 2)
    {
        accumulated_turn_level = guandao_speed_turn_level(
                &portion1_speed_planner, upcoming_turn);
    }
```

Leave the fallback fixed-threshold classification for other route choices unchanged.

- [ ] **Step 6: Apply the shared continuous rate limiter to both routes**

Change:

```c
if(route_setting_choice == 0)
```

around `guandao_speed_rate_limit()` to:

```c
if(route_setting_choice == 0 || route_setting_choice == 2)
```

Keep the current `35.0` acceleration and `40.0` deceleration rates and the elapsed-time clamp unchanged.

- [ ] **Step 7: Run focused tests and verify GREEN**

Run:

```powershell
python -m unittest tests.test_portion3_shared_speed_planner tests.test_portion1_speed_planner tests.test_final_integration -v
```

Expected: all focused tests pass.

- [ ] **Step 8: Commit the implementation**

```powershell
git add -- code/guandao.c
git commit -m "fix: share portion speed planning"
```

### Task 3: Full regression and delivery

**Files:**
- Verify: `code/guandao.c`
- Verify: `tests/test_portion3_start_rezero.py`
- Verify: `tests/test_portion3_save_consistency.py`
- Verify: all files under `tests/`

- [ ] **Step 1: Run the complete automated suite**

Run:

```powershell
python -m unittest discover -s tests -v
```

Expected: all tests pass, including Portion3 startup rezero, route-save consistency, endpoint protection, Portion1 reverse behavior, and rear PWM slew.

- [ ] **Step 2: Verify formatting and encoding**

Run:

```powershell
git diff --check
```

Then decode every modified source and test file as UTF-8 with Python. Expected: no decode errors and no whitespace errors.

- [ ] **Step 3: Review the final diff and commits**

Run:

```powershell
git status --short
git diff final...HEAD --stat
git log --oneline final..HEAD
```

Expected: only the approved design, plan, tests, and `code/guandao.c` changes are present.

- [ ] **Step 4: Fast-forward final and repeat the full test suite**

From the main worktree:

```powershell
git merge --ff-only codex/shared-speed-planner
python -m unittest discover -s tests -v
```

Expected: fast-forward succeeds without modifying unrelated dirty files, and all tests pass again.

- [ ] **Step 5: Push final**

```powershell
git push origin final
```

Expected: `origin/final` advances to the implementation commit.
