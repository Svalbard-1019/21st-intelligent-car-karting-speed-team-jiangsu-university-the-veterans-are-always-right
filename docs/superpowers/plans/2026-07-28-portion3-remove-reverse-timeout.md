# Portion3 Remove Reverse Timeout Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the fixed 39-second stop condition from Portion3 direct reverse while preserving endpoint and route-overrun stops.

**Architecture:** Keep the existing reverse state machine and its geometric stop helpers unchanged. Remove only the Portion3 timeout state, constant, initialization, and stop predicate; cover the behavior with a source-level integration regression test plus the existing host-side stop-helper tests.

**Tech Stack:** Embedded C99, Python `unittest`, host GCC

---

### Task 1: Remove the Portion3 direct-reverse time stop

**Files:**
- Modify: `tests/test_portion3_direct_reverse.py`
- Modify: `code/guandao.c:96,277,300,346,482-490`

- [ ] **Step 1: Write the failing regression test**

Rename `test_reverse_update_uses_reverse_heading_and_protected_stop` to
`test_reverse_update_has_geometric_stops_without_time_limit` and replace its
timeout assertion with:

```python
self.assertNotIn("PORTION3_DIRECT_REVERSE_MAX_MS", self.guandao_c)
self.assertNotIn("portion3_direct_reverse_start_ms", self.guandao_c)
self.assertIn("portion3_reverse_safety_stop(", update_body)
self.assertIn("portion3_reverse_should_stop(", update_body)
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```powershell
python -m unittest tests.test_portion3_direct_reverse.Portion3DirectReverseIntegrationTests.test_reverse_update_has_geometric_stops_without_time_limit -v
```

Expected: FAIL because `PORTION3_DIRECT_REVERSE_MAX_MS` and
`portion3_direct_reverse_start_ms` still exist in `code/guandao.c`.

- [ ] **Step 3: Implement the minimal production change**

Delete:

```c
static uint32 portion3_direct_reverse_start_ms = 0;
#define PORTION3_DIRECT_REVERSE_MAX_MS       (GUANDAO_REVERSE_MAX_MS * 6u)
portion3_direct_reverse_start_ms = 0;
portion3_direct_reverse_start_ms = system_getval_ms();
```

Change the stop predicate to:

```c
if(portion3_reverse_should_stop(
        portion3_direct_reverse_final_dist, final_yaw_error,
        portion3_direct_reverse_route_index, length)
        || portion3_reverse_safety_stop(
                portion3_direct_reverse_travelled,
                portion3_direct_reverse_route_length,
                PORTION3_DIRECT_REVERSE_OVERRUN))
{
    stop_requested = 1;
}
```

- [ ] **Step 4: Run focused and full verification**

Run:

```powershell
python -m unittest tests.test_portion3_direct_reverse -v
python -m unittest discover -s tests -p "test_*.py"
git diff --check
```

Expected: focused suite passes, all discovered tests pass, and
`git diff --check` exits successfully.

- [ ] **Step 5: Commit the implementation**

```powershell
git add -- code/guandao.c tests/test_portion3_direct_reverse.py
git commit -m "fix: remove portion3 reverse time limit"
```
