# Fixed Odometry Sampling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent high-speed INS distance loss by moving subject-one odometry input to the existing fixed 10 ms rear encoder sampler and reducing blocking diagnostics/display work.

**Architecture:** The existing CCU61_CH0 10 ms task remains the only TIM2 encoder sampler. It accumulates signed pulses in an interrupt-owned pending counter; `update_state()` atomically consumes that counter and integrates all accumulated distance, regardless of main-loop delay. Diagnostics expose sample interval, raw 10 ms delta, rejected samples, pending pulses, and main-loop interval while serial and screen output are rate-limited.

**Tech Stack:** TASKING C99, TC264 CCU6 interrupts, Seekfree encoder/UART/IPS200 drivers.

---

### Task 1: Add fixed-sampler odometry diagnostics

**Files:**
- Modify: `code/rear_motor/rear_motor.c`
- Modify: `code/rear_motor/rear_motor.h`

- [ ] Add volatile pending-pulse, rejected-sample, rejected-pulse, and maximum-sample-delta state.
- [ ] Update these values only from `rear_motor_encoder_update_10ms()`.
- [ ] Add an interrupt-safe `rear_motor_take_odometry_pulses()` consumer and read-only diagnostic getters.
- [ ] Reset all new state in `rear_motor_init()` and odometer reset paths.
- [ ] Run `git diff --check` and verify all new declarations have one definition.

### Task 2: Move INS distance integration to the fixed sampler

**Files:**
- Modify: `code/guandao.c`

- [ ] Replace `Encoder_Get(ecd)` inside `update_state()` with `rear_motor_take_odometry_pulses()`.
- [ ] Mirror the single trusted left-wheel pulse count into `ecd->delta_l` and `ecd->delta_r` for existing display/debug consumers.
- [ ] Preserve `ONE_TICK_DISTANCE`, forward/reverse signs, Yaw integration, route recording, pursuit, GPS, and parking behavior.
- [ ] Verify no direct `Encoder_Get(&guandao_ecd)` remains in subject-one record/trace paths.

### Task 3: Bound diagnostic and display load

**Files:**
- Modify: `user/cpu0_main.c`

- [ ] Replace the oversized AUTO line with a compact timing/odometry/control line containing `dt`, `raw`, `pending`, `drop`, `dropPulse`, `idx`, `x`, `y`, `yaw`, `rawSteer`, `finalSteer`, `actualSteer`, `targetSpeed`, and `actualSpeed`.
- [ ] Keep the 200 ms serial period and ensure the formatted line fits a small fixed buffer.
- [ ] Add a 200 ms screen refresh gate around record/automatic diagnostic pages.
- [ ] Remove no control calculations and preserve serial menu behavior.

### Task 4: Remove pursuit-loop screen I/O and verify

**Files:**
- Modify: `code/guandao.c`

- [ ] Remove the unconditional `ips200_show_float()` call from `pursuit_contral_mode()`.
- [ ] Run `git diff --check` on all modified files.
- [ ] Check modified C/H files for UTF-8 BOM bytes.
- [ ] Search for direct subject-one TIM2 reads and confirm the fixed sampler is the sole hardware reader during GUANDAO mode.
- [ ] Commit only the plan and intended C/H files, then push `codex/reverse-parking-test`.
