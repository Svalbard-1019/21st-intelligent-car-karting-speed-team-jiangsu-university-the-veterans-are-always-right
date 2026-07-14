#ifndef CODE_AUTO_PARK_PLAN_H_
#define CODE_AUTO_PARK_PLAN_H_

#define AUTO_PARK_MAX_ROUTES 4

typedef struct {
    float x;
    float y;
    float heading;
} AutoParkPose;

typedef struct {
    float x;
    float y;
} AutoParkPoint;

typedef struct {
    int is_forward;
    int is_straight;
    float distance;
    float start_heading;
    float steer_angle;
    float turn_radius;
} AutoParkRoute;

typedef struct {
    AutoParkPose start;
    AutoParkPose target;
    AutoParkPose end;
    AutoParkRoute routes[AUTO_PARK_MAX_ROUTES];
    int route_count;
    int valid;
    float score;
} AutoParkPlan;

float auto_park_normalize_deg(float angle);
AutoParkPoint auto_park_start_translation(AutoParkPose recorded_start, AutoParkPose actual_start);
int auto_park_reverse_safety_stop(int terminal_crossed, int route_near_end,
                                  float travelled, float planned_distance, float margin);
AutoParkPose auto_park_move_pose(AutoParkPose pose, const AutoParkRoute *route, float distance);
AutoParkPose auto_park_simulate_plan(AutoParkPose start, const AutoParkRoute *routes, int route_count);
int auto_park_build_plan(const AutoParkPose *start, const AutoParkPose *target, AutoParkPlan *out);

#endif
