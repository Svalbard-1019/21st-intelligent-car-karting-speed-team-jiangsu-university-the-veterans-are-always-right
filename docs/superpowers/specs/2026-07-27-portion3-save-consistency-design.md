# Portion 3 Save Consistency Design

## Problem

After a Portion 3 route is saved, the first autonomous run can follow a route that differs from the saved route. A reset makes the next run closer to the intended route, but the Flash reload also drops one valid point.

The save path reverses the live RAM route before writing it. Recording mode then remains active, so the current pose can be appended to that reversed route after the save. The first run consumes the polluted RAM route. A reset reloads Flash and removes the appended point, which explains the first-run/second-run difference. Separately, the Flash reader subtracts one from the stored route length even though the writer stores the exact transformed length.

## Approved Changes

1. Add a Portion 3 recording-session save lock.
   - A successful Portion 3 save marks the current recording session as saved.
   - While locked, `guandao_recode()` must not update odometry, average GPS, or append route points.
   - Both KEY4 and SBUS CH3 save paths activate the lock.
   - Entering any recording menu choice starts a new recording session and clears the lock.

2. Reload the exact Portion 3 Flash length.
   - `Flash_Read_portion_3points()` uses `stored_length` directly.
   - Continuation-page metadata must describe the exact stored length and continuation count.
   - Invalid metadata still rejects the route.

3. Change the Portion 3 serial firmware marker from `final2` to `p3save1` so test logs identify this firmware.

## Out of Scope

- Do not change the 800-point capacity or two-page Flash layout.
- Do not tune Portion 3 pursuit, steering, speed, odometry, or IMU parameters.
- Do not change Portion 1 behavior.

## Verification

Automated source regression tests will verify the session lock, both save inputs, recording-menu reset, exact Flash length, continuation validation, and serial marker. The complete Python test suite must pass before integration. The embedded TASKING build remains an ADS-side verification because that toolchain is not installed locally.
