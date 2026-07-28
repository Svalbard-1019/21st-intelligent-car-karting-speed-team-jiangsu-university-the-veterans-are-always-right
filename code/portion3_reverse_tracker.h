#ifndef CODE_PORTION3_REVERSE_TRACKER_H_
#define CODE_PORTION3_REVERSE_TRACKER_H_

#include <math.h>

#ifndef PORTION3_REVERSE_PI
#define PORTION3_REVERSE_PI 3.14159265358979323846f
#endif

static inline float portion3_reverse_normalize_angle(float angle_deg)
{
    while(angle_deg > 180.0f) angle_deg -= 360.0f;
    while(angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

static inline float portion3_reverse_motion_heading(float yaw_deg)
{
    return portion3_reverse_normalize_angle(yaw_deg + 180.0f);
}

static inline int portion3_reverse_initial_index(int route_length)
{
    return (route_length >= 3) ? 1 : 0;
}

static inline float portion3_reverse_steering(
        float heading_error_deg,
        float target_distance_m,
        float wheel_base_m,
        float gain,
        float steering_limit_deg)
{
    float target_steering;

    if(target_distance_m < 0.12f) target_distance_m = 0.12f;
    target_steering = gain * atan2f(
            2.0f * wheel_base_m
                    * sinf(heading_error_deg / 180.0f * PORTION3_REVERSE_PI),
            target_distance_m) / PORTION3_REVERSE_PI * 180.0f;
    target_steering = -target_steering;
    if(target_steering > steering_limit_deg) target_steering = steering_limit_deg;
    if(target_steering < -steering_limit_deg) target_steering = -steering_limit_deg;
    return target_steering;
}

static inline int portion3_reverse_should_stop(
        float final_distance_m,
        float final_yaw_error_deg,
        int route_index,
        int route_length)
{
    return route_length >= 3
            && route_index >= route_length - 1
            && final_distance_m <= 0.15f
            && fabsf(final_yaw_error_deg) <= 10.0f;
}

static inline int portion3_reverse_safety_stop(
        float travelled_m,
        float route_length_m,
        float overrun_m)
{
    return route_length_m > 0.0f
            && travelled_m > route_length_m + overrun_m;
}

#endif