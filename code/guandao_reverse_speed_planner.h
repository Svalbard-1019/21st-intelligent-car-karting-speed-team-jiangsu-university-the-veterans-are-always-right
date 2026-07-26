#ifndef GUANDAO_REVERSE_SPEED_PLANNER_H
#define GUANDAO_REVERSE_SPEED_PLANNER_H

typedef struct
{
    float cruise_units;
    float fine_units;
} guandao_reverse_speed_plan_t;

static inline guandao_reverse_speed_plan_t guandao_reverse_speed_plan(float menu_speed_units)
{
    guandao_reverse_speed_plan_t plan;

    if(menu_speed_units < -40.0f)
    {
        menu_speed_units = -40.0f;
    }
    if(menu_speed_units > -2.0f)
    {
        menu_speed_units = -2.0f;
    }

    plan.cruise_units = menu_speed_units;
    plan.fine_units = menu_speed_units * 0.75f;
    if(plan.fine_units < -3.0f)
    {
        plan.fine_units = -3.0f;
    }

    return plan;
}

#endif
