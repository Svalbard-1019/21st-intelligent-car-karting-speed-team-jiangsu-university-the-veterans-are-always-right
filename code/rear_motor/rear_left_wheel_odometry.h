#ifndef CODE_REAR_MOTOR_REAR_LEFT_WHEEL_ODOMETRY_H_
#define CODE_REAR_MOTOR_REAR_LEFT_WHEEL_ODOMETRY_H_

#include <stdint.h>

#define REAR_ODOMETRY_DEG_TO_RAD 0.01745329251994329577f

typedef struct
{
    uint8_t initialized;
    float last_yaw_deg;
} rear_left_wheel_odometry_t;

static inline float rear_odometry_normalize_delta_deg(float delta_deg)
{
    while(delta_deg > 180.0f) delta_deg -= 360.0f;
    while(delta_deg < -180.0f) delta_deg += 360.0f;
    return delta_deg;
}

static inline void rear_left_wheel_odometry_reset(
        rear_left_wheel_odometry_t *odometry)
{
    odometry->initialized = 0u;
    odometry->last_yaw_deg = 0.0f;
}

static inline float rear_left_wheel_to_center_distance(
        rear_left_wheel_odometry_t *odometry,
        float left_wheel_distance,
        float yaw_deg,
        float track_width)
{
    float yaw_delta_deg;
    float yaw_delta_rad;

    if(!odometry->initialized)
    {
        odometry->initialized = 1u;
        odometry->last_yaw_deg = yaw_deg;
        return left_wheel_distance;
    }

    yaw_delta_deg = rear_odometry_normalize_delta_deg(
            yaw_deg - odometry->last_yaw_deg);
    odometry->last_yaw_deg = yaw_deg;
    yaw_delta_rad = yaw_delta_deg * REAR_ODOMETRY_DEG_TO_RAD;

    return left_wheel_distance - track_width * 0.5f * yaw_delta_rad;
}

#endif
