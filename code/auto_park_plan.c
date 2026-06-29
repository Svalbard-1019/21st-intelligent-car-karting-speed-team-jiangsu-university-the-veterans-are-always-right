#include "auto_park_plan.h"

#include <math.h>
#include <string.h>

#ifndef AUTO_PARK_PI
#define AUTO_PARK_PI 3.14159265358979323846f
#endif

#ifndef AUTO_PARK_WHEEL_BASE
#define AUTO_PARK_WHEEL_BASE 0.724f
#endif

#ifndef AUTO_PARK_STEER_LIMIT_DEG
#define AUTO_PARK_STEER_LIMIT_DEG 40.0f
#endif

#define AUTO_PARK_MIN_RADIUS (AUTO_PARK_WHEEL_BASE / tanf(AUTO_PARK_STEER_LIMIT_DEG * AUTO_PARK_PI / 180.0f))
#define AUTO_PARK_BIG_SCORE  1000000000.0f

typedef struct {
    float x;
    float y;
} AutoParkPoint;

static float auto_park_absf(float v)
{
    return v < 0.0f ? -v : v;
}

static float auto_park_sign(float v)
{
    return v >= 0.0f ? 1.0f : -1.0f;
}

float auto_park_normalize_deg(float angle)
{
    while(angle > 180.0f) angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}

static AutoParkPoint auto_park_dir(float heading)
{
    float h = heading * AUTO_PARK_PI / 180.0f;
    AutoParkPoint p = {sinf(h), cosf(h)};
    return p;
}

static AutoParkPoint auto_park_right(float heading)
{
    float h = heading * AUTO_PARK_PI / 180.0f;
    AutoParkPoint p = {cosf(h), -sinf(h)};
    return p;
}

static void auto_park_make_straight(AutoParkRoute *route, int is_forward, float distance, float heading)
{
    route->is_forward = is_forward;
    route->is_straight = 1;
    route->distance = distance;
    route->start_heading = auto_park_normalize_deg(heading);
    route->steer_angle = 0.0f;
    route->turn_radius = 0.0f;
}

static void auto_park_make_turn(AutoParkRoute *route, int is_forward, float distance,
                                float heading, float steer_sign, float radius)
{
    route->is_forward = is_forward;
    route->is_straight = 0;
    route->distance = distance;
    route->start_heading = auto_park_normalize_deg(heading);
    route->steer_angle = steer_sign * atanf(AUTO_PARK_WHEEL_BASE / radius) * 180.0f / AUTO_PARK_PI;
    route->turn_radius = radius;
}

AutoParkPose auto_park_move_pose(AutoParkPose pose, const AutoParkRoute *route, float distance)
{
    AutoParkPoint dir = auto_park_dir(pose.heading);

    if(distance < 0.0f) distance = 0.0f;
    if(distance > route->distance) distance = route->distance;

    if(route->is_straight || auto_park_absf(route->steer_angle) < 0.001f || route->turn_radius <= 0.001f)
    {
        float move_sign = route->is_forward ? 1.0f : -1.0f;
        pose.x += move_sign * distance * dir.x;
        pose.y += move_sign * distance * dir.y;
        return pose;
    }

    float steer_sign = route->steer_angle >= 0.0f ? 1.0f : -1.0f;
    float move_sign = route->is_forward ? 1.0f : -1.0f;
    float heading_delta = move_sign * steer_sign * distance / route->turn_radius;
    AutoParkPoint right = auto_park_right(pose.heading);
    AutoParkPoint icr = {
        pose.x + steer_sign * route->turn_radius * right.x,
        pose.y + steer_sign * route->turn_radius * right.y
    };
    float vx = pose.x - icr.x;
    float vy = pose.y - icr.y;
    float c = cosf(-heading_delta);
    float s = sinf(-heading_delta);

    pose.x = icr.x + vx * c - vy * s;
    pose.y = icr.y + vx * s + vy * c;
    pose.heading = auto_park_normalize_deg(pose.heading + heading_delta * 180.0f / AUTO_PARK_PI);
    return pose;
}

AutoParkPose auto_park_simulate_plan(AutoParkPose start, const AutoParkRoute *routes, int route_count)
{
    AutoParkPose pose = start;
    for(int i = 0; i < route_count; i++)
    {
        pose.heading = routes[i].start_heading;
        pose = auto_park_move_pose(pose, &routes[i], routes[i].distance);
    }
    return pose;
}

static float auto_park_total_distance(const AutoParkRoute *routes, int route_count)
{
    float total = 0.0f;
    for(int i = 0; i < route_count; i++) total += routes[i].distance;
    return total;
}

static void auto_park_try_candidate(const AutoParkPose *start, const AutoParkPose *target,
                                    const AutoParkRoute *routes, int route_count,
                                    float extra_error, AutoParkPlan *best)
{
    AutoParkPose end = auto_park_simulate_plan(*start, routes, route_count);
    float dx = end.x - target->x;
    float dy = end.y - target->y;
    float dh = auto_park_normalize_deg(end.heading - target->heading);
    float score = sqrtf(dx * dx + dy * dy) * 80.0f
                + auto_park_absf(dh) * 20.0f
                + extra_error * 200.0f
                + auto_park_total_distance(routes, route_count);

    if(score < best->score)
    {
        best->valid = 1;
        best->score = score;
        best->route_count = route_count;
        best->end = end;
        for(int i = 0; i < route_count; i++) best->routes[i] = routes[i];
    }
}

static void auto_park_try_reverse_straight(const AutoParkPose *start, const AutoParkPose *target,
                                           AutoParkPlan *best)
{
    AutoParkPoint dir = auto_park_dir(start->heading);
    float dx = target->x - start->x;
    float dy = target->y - start->y;
    float along = dx * dir.x + dy * dir.y;
    float lateral = dx * (-dir.y) + dy * dir.x;
    float dh = auto_park_normalize_deg(target->heading - start->heading);

    if(along < -0.05f && auto_park_absf(lateral) < 0.08f && auto_park_absf(dh) < 4.0f)
    {
        AutoParkRoute route;
        auto_park_make_straight(&route, 0, -along, start->heading);
        auto_park_try_candidate(start, target, &route, 1, auto_park_absf(lateral), best);
    }
}

static void auto_park_search_turn_straight_turn(const AutoParkPose *start, const AutoParkPose *target,
                                                AutoParkPlan *best)
{
    AutoParkRoute routes[AUTO_PARK_MAX_ROUTES];
    AutoParkPoint final_dir = auto_park_dir(target->heading);

    for(float final_dist = 0.25f; final_dist <= 1.45f; final_dist += 0.10f)
    {
        AutoParkPose p3 = {
            target->x + final_dist * final_dir.x,
            target->y + final_dist * final_dir.y,
            target->heading
        };

        for(float h1 = -180.0f; h1 <= 180.0f; h1 += 8.0f)
        {
            float d01 = auto_park_normalize_deg(h1 - start->heading);
            float d13 = auto_park_normalize_deg(target->heading - h1);
            if(auto_park_absf(d01) < 3.0f || auto_park_absf(d13) < 3.0f) continue;

            for(float r2 = AUTO_PARK_MIN_RADIUS; r2 <= 3.05f; r2 += 0.25f)
            {
                float steer2_sign = -auto_park_sign(d13);
                float arc2 = auto_park_absf(d13 * AUTO_PARK_PI / 180.0f * r2);
                AutoParkRoute reverse_arc;
                AutoParkRoute inverse_arc;
                auto_park_make_turn(&inverse_arc, 1, arc2, target->heading, steer2_sign, r2);
                auto_park_make_turn(&reverse_arc, 0, arc2, h1, steer2_sign, r2);
                AutoParkPose p2 = auto_park_move_pose(p3, &inverse_arc, arc2);

                for(float r1 = AUTO_PARK_MIN_RADIUS; r1 <= 3.05f; r1 += 0.25f)
                {
                    float steer1_sign = auto_park_sign(d01);
                    float arc1 = auto_park_absf(d01 * AUTO_PARK_PI / 180.0f * r1);
                    AutoParkRoute forward_arc;
                    AutoParkPose p1;
                    AutoParkPoint dir1;
                    float vx;
                    float vy;
                    float straight;
                    float lateral;

                    auto_park_make_turn(&forward_arc, 1, arc1, start->heading, steer1_sign, r1);
                    p1 = auto_park_move_pose(*start, &forward_arc, arc1);
                    dir1 = auto_park_dir(h1);
                    vx = p2.x - p1.x;
                    vy = p2.y - p1.y;
                    straight = vx * dir1.x + vy * dir1.y;
                    lateral = vx * (-dir1.y) + vy * dir1.x;
                    if(straight < 0.05f || auto_park_absf(lateral) > 0.08f) continue;

                    auto_park_make_turn(&routes[0], 1, arc1, start->heading, steer1_sign, r1);
                    auto_park_make_straight(&routes[1], 1, straight, h1);
                    auto_park_make_turn(&routes[2], 0, arc2, h1, steer2_sign, r2);
                    auto_park_make_straight(&routes[3], 0, final_dist, target->heading);
                    auto_park_try_candidate(start, target, routes, 4, auto_park_absf(lateral), best);
                }
            }
        }
    }
}

int auto_park_build_plan(const AutoParkPose *start, const AutoParkPose *target, AutoParkPlan *out)
{
    if(start == 0 || target == 0 || out == 0) return 0;

    memset(out, 0, sizeof(*out));
    out->start = *start;
    out->target = *target;
    out->score = AUTO_PARK_BIG_SCORE;

    auto_park_try_reverse_straight(start, target, out);
    auto_park_search_turn_straight_turn(start, target, out);

    return out->valid;
}
