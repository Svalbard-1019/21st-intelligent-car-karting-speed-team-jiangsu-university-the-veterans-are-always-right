#ifndef CODE_REAR_MOTOR_REAR_REVERSE_PWM_FLOOR_H_
#define CODE_REAR_MOTOR_REAR_REVERSE_PWM_FLOOR_H_

static inline float rear_reverse_apply_startup_pwm_floor(
        float target_mps, float actual_mps, float requested_pwm,
        float startup_speed_mps, float startup_pwm_floor)
{
    if(target_mps < -0.01f
            && actual_mps > -startup_speed_mps
            && requested_pwm < 0.0f
            && requested_pwm > -startup_pwm_floor)
    {
        return -startup_pwm_floor;
    }
    return requested_pwm;
}

#endif
