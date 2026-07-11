#ifndef CODE_REAR_MOTOR_REAR_ODOMETRY_BUFFER_H_
#define CODE_REAR_MOTOR_REAR_ODOMETRY_BUFFER_H_

#include <stdint.h>

typedef struct
{
    volatile int32_t pending_pulses;
    volatile int32_t total_pulses;
    volatile uint32_t rejected_samples;
    volatile int32_t rejected_pulses;
    volatile int32_t max_abs_sample;
    volatile int32_t last_sample;
} rear_odometry_buffer_t;

static inline void rear_odometry_buffer_init(rear_odometry_buffer_t *buffer)
{
    buffer->pending_pulses = 0;
    buffer->total_pulses = 0;
    buffer->rejected_samples = 0;
    buffer->rejected_pulses = 0;
    buffer->max_abs_sample = 0;
    buffer->last_sample = 0;
}

static inline void rear_odometry_buffer_add(
        rear_odometry_buffer_t *buffer, int32_t sample, int32_t sample_limit)
{
    int32_t abs_sample = (sample < 0) ? -sample : sample;

    buffer->last_sample = sample;
    if(abs_sample > buffer->max_abs_sample)
    {
        buffer->max_abs_sample = abs_sample;
    }
    if(abs_sample > sample_limit)
    {
        buffer->rejected_samples++;
        buffer->rejected_pulses += sample;
        return;
    }

    buffer->pending_pulses += sample;
    buffer->total_pulses += sample;
}

static inline int32_t rear_odometry_buffer_take(rear_odometry_buffer_t *buffer)
{
    int32_t pulses = buffer->pending_pulses;
    buffer->pending_pulses = 0;
    return pulses;
}

#endif
