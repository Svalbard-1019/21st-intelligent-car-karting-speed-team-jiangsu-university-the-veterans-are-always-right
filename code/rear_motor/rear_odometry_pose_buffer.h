#ifndef CODE_REAR_MOTOR_REAR_ODOMETRY_POSE_BUFFER_H_
#define CODE_REAR_MOTOR_REAR_ODOMETRY_POSE_BUFFER_H_

#include <stdint.h>

#define REAR_ODOMETRY_POSE_BUFFER_CAPACITY 64u

typedef struct
{
    int32_t pulses;
    float yaw_deg;
} rear_odometry_pose_sample_t;

typedef struct
{
    rear_odometry_pose_sample_t samples[REAR_ODOMETRY_POSE_BUFFER_CAPACITY];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
    volatile uint32_t merged_samples;
} rear_odometry_pose_buffer_t;

static inline void rear_odometry_pose_buffer_init(rear_odometry_pose_buffer_t *buffer)
{
    buffer->head = 0u;
    buffer->tail = 0u;
    buffer->count = 0u;
    buffer->merged_samples = 0u;
}

static inline void rear_odometry_pose_buffer_add(
        rear_odometry_pose_buffer_t *buffer, int32_t pulses, float yaw_deg)
{
    if(buffer->count >= REAR_ODOMETRY_POSE_BUFFER_CAPACITY)
    {
        uint8_t last = (buffer->head == 0u)
                ? (uint8_t)(REAR_ODOMETRY_POSE_BUFFER_CAPACITY - 1u)
                : (uint8_t)(buffer->head - 1u);
        buffer->samples[last].pulses += pulses;
        buffer->samples[last].yaw_deg = yaw_deg;
        buffer->merged_samples++;
        return;
    }

    buffer->samples[buffer->head].pulses = pulses;
    buffer->samples[buffer->head].yaw_deg = yaw_deg;
    buffer->head = (uint8_t)((buffer->head + 1u) % REAR_ODOMETRY_POSE_BUFFER_CAPACITY);
    buffer->count++;
}

static inline uint8_t rear_odometry_pose_buffer_take(
        rear_odometry_pose_buffer_t *buffer, rear_odometry_pose_sample_t *sample)
{
    if(buffer->count == 0u) return 0u;

    *sample = buffer->samples[buffer->tail];
    buffer->tail = (uint8_t)((buffer->tail + 1u) % REAR_ODOMETRY_POSE_BUFFER_CAPACITY);
    buffer->count--;
    return 1u;
}

#endif
