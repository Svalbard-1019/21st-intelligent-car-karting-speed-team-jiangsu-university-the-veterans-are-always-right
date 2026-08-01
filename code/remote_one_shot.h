#ifndef REMOTE_ONE_SHOT_H
#define REMOTE_ONE_SHOT_H

typedef struct
{
    unsigned char armed;
} remote_one_shot_t;

static inline void remote_one_shot_reset(remote_one_shot_t *state)
{
    if(state != 0) state->armed = 1u;
}

static inline unsigned char remote_one_shot_update(
        remote_one_shot_t *state, unsigned char pressed)
{
    if(state == 0) return 0u;
    if(!pressed)
    {
        state->armed = 1u;
        return 0u;
    }
    if(state->armed)
    {
        state->armed = 0u;
        return 1u;
    }
    return 0u;
}

#endif
