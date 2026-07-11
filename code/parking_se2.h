#ifndef CODE_PARKING_SE2_H_
#define CODE_PARKING_SE2_H_

#include <math.h>

#define PARKING_SE2_PI 3.14159265358979323846f

static inline float parking_se2_normalize_angle(float angle_deg)
{
    while(angle_deg > 180.0f) angle_deg -= 360.0f;
    while(angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

/* Parking entry is an axis: headings separated by 180 degrees describe the
 * same line. Return the smallest rotation that aligns the two entry axes. */
static inline float parking_se2_axis_delta(
        float recorded_heading_deg, float runtime_heading_deg)
{
    float delta = parking_se2_normalize_angle(
            runtime_heading_deg - recorded_heading_deg);
    if(delta > 90.0f) delta -= 180.0f;
    if(delta < -90.0f) delta += 180.0f;
    return delta;
}

/*
 * Transform a taught point from the recorded parking-entry frame into the
 * current run's parking-entry frame. The vehicle convention is x=sin(yaw),
 * y=cos(yaw), so yaw zero points along local +Y.
 */
static inline void parking_se2_transform_point(
        float recorded_origin_x, float recorded_origin_y, float recorded_heading_deg,
        float runtime_origin_x, float runtime_origin_y, float runtime_heading_deg,
        float source_x, float source_y, float *target_x, float *target_y)
{
    float recorded_rad = recorded_heading_deg * PARKING_SE2_PI / 180.0f;
    float runtime_rad = runtime_heading_deg * PARKING_SE2_PI / 180.0f;
    float dx = source_x - recorded_origin_x;
    float dy = source_y - recorded_origin_y;
    float local_x = dx * cosf(recorded_rad) - dy * sinf(recorded_rad);
    float local_y = dx * sinf(recorded_rad) + dy * cosf(recorded_rad);

    *target_x = runtime_origin_x
            + local_x * cosf(runtime_rad) + local_y * sinf(runtime_rad);
    *target_y = runtime_origin_y
            - local_x * sinf(runtime_rad) + local_y * cosf(runtime_rad);
}

static inline float parking_se2_transform_heading(
        float source_heading_deg, float recorded_heading_deg, float runtime_heading_deg)
{
    return parking_se2_normalize_angle(
            source_heading_deg - recorded_heading_deg + runtime_heading_deg);
}

#endif
