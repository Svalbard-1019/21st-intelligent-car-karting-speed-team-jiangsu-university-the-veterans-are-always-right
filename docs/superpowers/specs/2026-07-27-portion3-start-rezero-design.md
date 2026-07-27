# Portion 3 Start Rezero Design

## Problem

The `p3save1` log contains two Portion 3 runs over the same 393-point route. The first run diverges at startup, while the run after an MCU reset completes the route.

The route data is identical in both runs. In the first run, `Yaw_1` changes from 0 degrees to 7.1 degrees after only 13 encoder pulses, about 6 mm of travel. That motion cannot physically produce the reported yaw with the measured steering angle and 0.724 m wheelbase. An MCU reset fixes the behavior because it runs the complete IMU offset calibration and clears rear odometry runtime state. `portion3_return_reset()` currently only assigns `Yaw_1 = 0.0f` and resets the high-level pose.

## Approved Behavior

1. Starting Portion 3 begins a nonblocking one-second Z-axis gyro calibration.
   - Calibration uses 125 samples at the existing approximately 8 ms IMU update cadence.
   - While calibration is active, `Yaw_1` remains zero and normal yaw integration is suspended.
   - The final average becomes `Gyro_Offset.Zdata`.

2. Portion 3 tracking is gated during calibration.
   - Rear speed commands and steering commands remain zero.
   - Route index and pose are not advanced.
   - Debug stop reason 12 identifies the calibration wait.

3. Rear odometry runtime state is reset at the coordinate-frame boundary.
   - A public rear-motor API atomically clears the pose sample queue and total pulse origin.
   - It synchronizes the encoder baseline to the current hardware count.
   - The reset runs when Portion 3 is selected and again when gyro calibration finishes, excluding all pre-start samples.

4. The Portion 3 serial marker becomes `p3save2`.
   - Add `p3Init`, `gzRaw`, and `gzOff` fields.
   - `p3Init=1` means calibration is still active.

## Safety and Scope

- The vehicle must remain stationary during the one-second calibration.
- Calibration is nonblocking; no five-second delay is added to the menu or main loop.
- Portion 1, route Flash format, 800-point storage, speed planning, pursuit tuning, and steering gains are unchanged.
- Existing boot-time IMU calibration remains unchanged.

## Verification

Source regression tests will verify the nonblocking sample accumulator, calibration gate, odometry reset boundary, and serial marker. The full Python test suite must pass. ADS/TASKING Clean/Build and the physical first-run test remain hardware-side verification.
