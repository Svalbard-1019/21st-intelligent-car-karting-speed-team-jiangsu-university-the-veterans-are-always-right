#ifndef CODE_PORTION1_PRECOAST_H_
#define CODE_PORTION1_PRECOAST_H_

#define PORTION1_PRECOAST_BASE_THRESHOLD 30.0f
#define PORTION1_PRECOAST_TARGET_MPS 1.0f
#define PORTION1_PRECOAST_RELEASE_MPS 1.15f
#define PORTION1_PRECOAST_DECEL_MPS2 0.60f
#define PORTION1_PRECOAST_RESPONSE_S 0.20f
#define PORTION1_PRECOAST_MARGIN_M 0.50f

typedef struct
{
    unsigned char latched;
    unsigned char coast_output;
    float remaining_m;
    float trigger_m;
} portion1_precoast_t;

static inline void portion1_precoast_reset(portion1_precoast_t *state)
{
    state->latched = 0u;
    state->coast_output = 0u;
    state->remaining_m = 0.0f;
    state->trigger_m = 0.0f;
}

static inline float portion1_precoast_trigger_m(float speed_mps)
{
    float braking = speed_mps * speed_mps
            - PORTION1_PRECOAST_TARGET_MPS * PORTION1_PRECOAST_TARGET_MPS;
    if(braking < 0.0f) braking = 0.0f;
    return braking / (2.0f * PORTION1_PRECOAST_DECEL_MPS2)
            + speed_mps * PORTION1_PRECOAST_RESPONSE_S
            + PORTION1_PRECOAST_MARGIN_M;
}

static inline void portion1_precoast_update(portion1_precoast_t *state,
        unsigned char route_valid, unsigned char forward_active,
        float base_speed_units, float actual_speed_mps, float remaining_m)
{
    if(!route_valid || !forward_active)
    {
        portion1_precoast_reset(state);
        return;
    }

    state->remaining_m = remaining_m;
    state->trigger_m = portion1_precoast_trigger_m(actual_speed_mps);
    if(!state->latched
            && base_speed_units > PORTION1_PRECOAST_BASE_THRESHOLD
            && actual_speed_mps > PORTION1_PRECOAST_RELEASE_MPS
            && remaining_m <= state->trigger_m)
    {
        state->latched = 1u;
    }
    state->coast_output = state->latched
            && actual_speed_mps > PORTION1_PRECOAST_RELEASE_MPS;
}

#endif
