#ifndef CODE_PORTION1_PARKING_BRAKE_H_
#define CODE_PORTION1_PARKING_BRAKE_H_

#define PORTION1_PARK_BRAKE_DECEL_MPS2          2.0f
#define PORTION1_PARK_BRAKE_RESPONSE_S          0.15f
#define PORTION1_PARK_BRAKE_MARGIN_M            0.20f
#define PORTION1_PARK_BRAKE_MIN_OVERSPEED_MPS   0.30f

static inline float portion1_parking_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static inline float portion1_parking_slowdown_distance(
        float speed_mps, float target_speed_mps)
{
    float speed = portion1_parking_absf(speed_mps);
    float target = portion1_parking_absf(target_speed_mps);
    float kinetic_distance;

    if(speed <= target) return 0.0f;
    kinetic_distance = (speed * speed - target * target)
            / (2.0f * PORTION1_PARK_BRAKE_DECEL_MPS2);
    return kinetic_distance + speed * PORTION1_PARK_BRAKE_RESPONSE_S
            + PORTION1_PARK_BRAKE_MARGIN_M;
}

static inline unsigned char portion1_parking_should_slowdown(
        float longitudinal_m, float speed_mps, float target_speed_mps,
        unsigned char already_requested)
{
    float speed = portion1_parking_absf(speed_mps);
    float target = portion1_parking_absf(target_speed_mps);

    if(already_requested || longitudinal_m >= 0.0f) return 0u;
    if(speed <= target + PORTION1_PARK_BRAKE_MIN_OVERSPEED_MPS) return 0u;
    return (-longitudinal_m <= portion1_parking_slowdown_distance(
            speed, target)) ? 1u : 0u;
}

#endif
