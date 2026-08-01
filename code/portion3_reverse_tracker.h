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

static inline int portion3_reverse_preview_index(
        int route_index,
        int route_length,
        int preview_steps)
{
    int preview_index;

    if(route_length <= 0) return 0;
    if(route_index < 0) route_index = 0;
    if(preview_steps < 0) preview_steps = 0;
    preview_index = route_index + preview_steps;
    if(preview_index >= route_length) preview_index = route_length - 1;
    return preview_index;
}

static inline int portion3_reverse_metric_preview_index(
        const float *route_m, int route_index, int route_length,
        float lookahead_m)
{
    int preview_index;
    float target_m;

    if(route_length <= 0) return 0;
    if(route_index < 0) route_index = 0;
    if(route_index >= route_length) route_index = route_length - 1;
    if(lookahead_m < 0.0f) lookahead_m = 0.0f;
    target_m = route_m[route_index] + lookahead_m;
    preview_index = route_index;
    while(preview_index < route_length - 1
            && route_m[preview_index] < target_m)
    {
        preview_index++;
    }
    return preview_index;
}

typedef struct
{
    unsigned char level;
} portion3_reverse_turn_state_t;

static inline void portion3_reverse_turn_reset(
        portion3_reverse_turn_state_t *state)
{
    state->level = 0u;
}

static inline unsigned char portion3_reverse_turn_level(
        portion3_reverse_turn_state_t *state, float heading_error_deg)
{
    float magnitude = fabsf(heading_error_deg);
    unsigned char level = state->level;

    if(level == 0u)
    {
        if(magnitude >= 45.0f) level = 3u;
        else if(magnitude >= 30.0f) level = 2u;
        else if(magnitude >= 15.0f) level = 1u;
    }
    else if(level == 1u)
    {
        if(magnitude >= 45.0f) level = 3u;
        else if(magnitude >= 30.0f) level = 2u;
        else if(magnitude < 10.0f) level = 0u;
    }
    else if(level == 2u)
    {
        if(magnitude >= 45.0f) level = 3u;
        else if(magnitude < 25.0f)
        {
            level = (magnitude >= 15.0f) ? 1u : 0u;
        }
    }
    else if(magnitude < 40.0f)
    {
        if(magnitude >= 30.0f) level = 2u;
        else if(magnitude >= 15.0f) level = 1u;
        else level = 0u;
    }

    state->level = level;
    return level;
}

static inline float portion3_reverse_curve_speed(
        float cruise_speed, float fine_speed,
        unsigned char turn_level, unsigned char fine_active)
{
    float ratio = 1.0f;
    float command;

    if(turn_level >= 3u) ratio = 0.55f;
    else if(turn_level == 2u) ratio = 0.70f;
    else if(turn_level == 1u) ratio = 0.85f;
    command = cruise_speed * ratio;
    if(fine_active && fabsf(fine_speed) < fabsf(command)) command = fine_speed;
    return command;
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
