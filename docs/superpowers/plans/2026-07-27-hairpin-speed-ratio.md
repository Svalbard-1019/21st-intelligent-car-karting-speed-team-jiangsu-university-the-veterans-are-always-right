# Portion1 Hairpin Speed Ratio Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Raise the portion1 hairpin speed ratio from 0.45 to 0.60 without introducing a fixed speed floor.

**Architecture:** Keep the existing route-speed pipeline and change its single hairpin ratio constant. Protect the behavior with the existing source-level speed-planner test so final approach and stopping logic remain independent.

**Tech Stack:** Embedded C, Python `unittest`, host GCC tests

---

### Task 1: Raise the hairpin ratio

**Files:**
- Modify: `tests/test_portion1_speed_planner.py`
- Modify: `code/guandao.c`
- Modify: `user/cpu0_main.c`

- [ ] **Step 1: Write the failing test**

Change the expected token in
`Portion1SpeedPlannerTests.test_portion1_uses_distance_window_and_keeps_portion3_isolated`:

```python
self.assertIn("GUANDAO_HAIRPIN_SPEED_RATIO    0.60f", source)
```

Keep the test assertion that the portion1 acceleration limit remains
`35.0f`; do not add a fixed curve-speed floor.

Change the expected serial marker to:

```python
"cfg=p1spd8",
```

- [ ] **Step 2: Run the focused test and verify it fails**

Run:

```powershell
python -m unittest tests.test_portion1_speed_planner.Portion1SpeedPlannerTests.test_portion1_uses_distance_window_and_keeps_portion3_isolated
```

Expected: failure because `code/guandao.c` still contains
`GUANDAO_HAIRPIN_SPEED_RATIO    0.45f` and `user/cpu0_main.c` still contains
`cfg=p1spd7`.

- [ ] **Step 3: Apply the minimal production change**

In `code/guandao.c`, change:

```c
#define GUANDAO_HAIRPIN_SPEED_RATIO    0.45f
```

to:

```c
#define GUANDAO_HAIRPIN_SPEED_RATIO    0.60f
```

In `user/cpu0_main.c`, change the AUTO serial marker:

```c
cfg=p1spd7
```

to:

```c
cfg=p1spd8
```

- [ ] **Step 4: Run the focused test and verify it passes**

Run the focused command from Step 2.

Expected: one test passes.

- [ ] **Step 5: Run complete verification**

Run:

```powershell
python -m unittest discover -s tests -p "test_*.py"
git diff --check
```

Expected: 49 tests pass and `git diff --check` reports no errors.

- [ ] **Step 6: Commit**

Stage only the design, plan, ratio constant, and associated test, then commit:

```powershell
git commit -m "fix: raise portion1 hairpin speed ratio"
```
