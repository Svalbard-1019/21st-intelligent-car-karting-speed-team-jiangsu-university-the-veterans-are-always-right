/*
 * Shared rear encoder distance calibration.
 *
 * This value comes from the measured travel-distance test and must be used by
 * both odometry and the rear speed loop. Keeping one definition prevents the
 * route coordinates and speed controller from using different distance scales.
 */
#ifndef CODE_REAR_ENCODER_CALIBRATION_H_
#define CODE_REAR_ENCODER_CALIBRATION_H_

#define REAR_ENCODER_METERS_PER_PULSE 0.000378f

#endif /* CODE_REAR_ENCODER_CALIBRATION_H_ */
