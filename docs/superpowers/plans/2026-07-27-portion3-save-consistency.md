# Portion 3 Save Consistency Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep the first Portion 3 run identical to the saved RAM/Flash route and preserve every saved point after reset.

**Architecture:** Add a recording-session lock in `guandao.c` that is set by both save inputs and cleared by a new menu recording session. Correct `flash.c` to load the exact saved length and validate the continuation page against that exact length. Identify the firmware in Portion 3 logs with `p3save1`.

**Tech Stack:** Infineon AURIX C firmware, Python `unittest` source regression tests, Git.

---

### Task 1: Add failing Portion 3 consistency regressions

**Files:**
- Create: `tests/test_portion3_save_consistency.py`

- [ ] **Step 1: Write failing tests**

Create tests that extract the relevant C function bodies and assert:

```python
self.assertIn("static uint8 guandao_record_saved = 0;", guandao_c)
self.assertIn("guandao_record_saved = 1;", recode_body)
self.assertLess(recode_body.index("if(guandao_record_saved) return;"),
                recode_body.index("update_state(p  , &guandao_ecd);"))
self.assertIn("guandao_record_saved = 0;", reset_body)
self.assertEqual(record_menu_body.count("guandao_record_session_reset();"), 1)
self.assertIn("portion_3.length_index = flash_clamp_route_length(stored_length);",
              flash_read_body)
self.assertNotIn("stored_length - 1", flash_read_body)
self.assertIn("cfg=p3save1", main_c)
```

- [ ] **Step 2: Verify the tests fail**

Run: `python -m unittest tests.test_portion3_save_consistency -v`

Expected: failures because the save lock and exact-length load are not implemented and the marker is still `final2`.

- [ ] **Step 3: Commit the red tests**

Run:

```text
git add tests/test_portion3_save_consistency.py
git commit -m "test: cover portion3 save consistency"
```

### Task 2: Lock recording after a Portion 3 save

**Files:**
- Modify: `code/guandao.c`
- Modify: `code/display.c`
- Test: `tests/test_portion3_save_consistency.py`

- [ ] **Step 1: Add the minimal lock state**

Add:

```c
static uint8 guandao_record_saved = 0;
```

After selecting the active route in `guandao_recode()`, return before state updates when the lock is set. After each Portion 3 save path, set the lock. In the SBUS path, return before any post-save `recode_waypoint()` call.

- [ ] **Step 2: Clear the lock for a new recording session**

Set `guandao_record_saved = 0;` in `guandao_record_session_reset()`. In the recording menu, select the route in the switch and then call `guandao_record_session_reset()` exactly once for every valid route selection.

- [ ] **Step 3: Verify the focused lifecycle tests pass**

Run: `python -m unittest tests.test_portion3_save_consistency.Portion3SaveConsistencyTests.test_save_locks_recording_until_new_session tests.test_portion3_save_consistency.Portion3SaveConsistencyTests.test_record_menu_starts_a_fresh_session -v`

Expected: 2 tests pass.

### Task 3: Preserve the exact Flash route length

**Files:**
- Modify: `code/flash.c`
- Test: `tests/test_portion3_save_consistency.py`

- [ ] **Step 1: Load the stored length without subtraction**

Replace:

```c
portion_3.length_index = flash_clamp_route_length(stored_length - 1);
```

with:

```c
portion_3.length_index = flash_clamp_route_length(stored_length);
```

Require the continuation header count to equal `continuation_count`.

- [ ] **Step 2: Verify the focused Flash test passes**

Run: `python -m unittest tests.test_portion3_save_consistency.Portion3SaveConsistencyTests.test_flash_reload_preserves_exact_saved_length -v`

Expected: 1 test passes.

### Task 4: Add the firmware marker and verify integration

**Files:**
- Modify: `user/cpu0_main.c`
- Test: `tests/test_portion3_save_consistency.py`

- [ ] **Step 1: Update the Portion 3 marker**

Change the `P3AUTO` marker from:

```c
cfg=final2
```

to:

```c
cfg=p3save1
```

- [ ] **Step 2: Run all tests**

Run: `python -m unittest discover -s tests -p 'test_*.py'`

Expected: all tests pass with zero failures.

- [ ] **Step 3: Inspect the scoped diff**

Run: `git diff final...HEAD -- code/guandao.c code/display.c code/flash.c user/cpu0_main.c tests/test_portion3_save_consistency.py`

Expected: only the approved lock, exact-length validation, marker, and regression tests are present.

- [ ] **Step 4: Commit implementation**

Run:

```text
git add code/guandao.c code/display.c code/flash.c user/cpu0_main.c
git commit -m "fix: keep portion3 save and reload routes consistent"
```

- [ ] **Step 5: Merge to final and push**

Merge the verified feature branch into `final`, rerun the complete test suite in the main worktree, and push `origin/final`.
