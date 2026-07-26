#ifndef CODE_GUANDAO_SPEED_PLANNER_H_
#define CODE_GUANDAO_SPEED_PLANNER_H_

typedef struct
{
    float command;
    unsigned char turn_level;
} guandao_speed_planner_t;

static inline void guandao_speed_planner_reset(
        guandao_speed_planner_t *planner, float initial_command)
{
    planner->command = initial_command;
    planner->turn_level = 0u;
}

static inline unsigned char guandao_speed_turn_level(
        guandao_speed_planner_t *planner, float turn_degrees)
{
    float magnitude = (turn_degrees < 0.0f) ? -turn_degrees : turn_degrees;
    unsigned char level = planner->turn_level;

    if(level == 0u)
    {
        if(magnitude >= 75.0f) level = 3u;
        else if(magnitude >= 45.0f) level = 2u;
        else if(magnitude >= 20.0f) level = 1u;
    }
    else if(level == 1u)
    {
        if(magnitude >= 75.0f) level = 3u;
        else if(magnitude >= 45.0f) level = 2u;
        else if(magnitude < 15.0f) level = 0u;
    }
    else if(level == 2u)
    {
        if(magnitude >= 75.0f) level = 3u;
        else if(magnitude < 35.0f)
        {
            level = (magnitude >= 15.0f) ? 1u : 0u;
        }
    }
    else
    {
        if(magnitude < 60.0f)
        {
            if(magnitude >= 35.0f) level = 2u;
            else if(magnitude >= 15.0f) level = 1u;
            else level = 0u;
        }
    }

    planner->turn_level = level;
    return level;
}

static inline float guandao_speed_rate_limit(
        guandao_speed_planner_t *planner, float requested,
        float elapsed_seconds, float acceleration_rate, float deceleration_rate)
{
    float delta;
    float limit;

    if(elapsed_seconds <= 0.0f) return planner->command;
    delta = requested - planner->command;
    limit = ((delta >= 0.0f) ? acceleration_rate : deceleration_rate)
            * elapsed_seconds;
    if(delta > limit) delta = limit;
    if(delta < -limit) delta = -limit;
    planner->command += delta;
    return planner->command;
}

#endif
