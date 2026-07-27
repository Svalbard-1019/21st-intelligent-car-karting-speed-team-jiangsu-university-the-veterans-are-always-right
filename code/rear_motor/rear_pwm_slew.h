#ifndef REAR_PWM_SLEW_H
#define REAR_PWM_SLEW_H

static inline int rear_pwm_limit_request_sign(float target_mps, int requested_pwm)
{
    if(target_mps > 0.0f && requested_pwm < 0) return 0;
    if(target_mps < 0.0f && requested_pwm > 0) return 0;
    return requested_pwm;
}

static inline int rear_pwm_slew_step(
        int current_pwm,
        int requested_pwm,
        int drive_limit,
        int release_limit)
{
    int diff = requested_pwm - current_pwm;
    int limit = drive_limit;

    if((current_pwm > 0 && requested_pwm >= 0 && requested_pwm < current_pwm)
            || (current_pwm < 0 && requested_pwm <= 0 && requested_pwm > current_pwm))
    {
        limit = release_limit;
    }

    if(diff > limit) diff = limit;
    if(diff < -limit) diff = -limit;
    return current_pwm + diff;
}

#endif
