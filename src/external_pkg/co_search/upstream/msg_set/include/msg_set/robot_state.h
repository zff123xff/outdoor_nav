#ifndef _ROBOT_STATE_H_
#define _ROBOT_STATE_H_

namespace fsm_major_state
{
enum FSM_MAJOR_STATE
{
    AUTOMATIC,
    ON_MOTOR,
    OFF_MOTOR,
    TAKEOFF,
    HOVER,
    MOVE_POSITION,
    LAND,
    FOLLOW_GOAL,
    STATIC_AVOIDANCE,
    DYNAMIC_AVOIDANCE,
    ORIGIN_ALIGN,
    COMPANION_TRACK,
    ATTACK_ONLY,
    SWARM_FORMATION,
    HORIZONTAL_SQUARE,
    DYNAMIC_SQUARE,
    VERTICLE_CIRCLE,
    ATTACK_BALL
};
static const char* fsm_major_state_strings[] = {
    "AUTOMATIC",   "ON_MOTOR",        "OFF_MOTOR",         "TAKEOFF",           "HOVER",           "MOVE_POSITION",
    "LAND",        "FOLLOW_GOAL",     "STATIC_AVOIDANCE",  "DYNAMIC_AVOIDANCE", "ORIGIN_ALIGN",    "COMPANION_TRACK",
    "ATTACK_ONLY", "SWARM_FORMATION", "HORIZONTAL_SQUARE", "DYNAMIC_SQUARE",    "VERTICLE_CIRCLE", "ATTACK_BALL"
};
}  // namespace fsm_major_state
namespace gui_ctrl_state
{
enum GUI_CTRL_STATE
{
    USELESS,
    START,
    STOP,
    TAKEOFF,
    HOVER,
    MOVE_POSITION,
    LAND,
    FOLLOW_GOAL,
    STATIC_AVOIDANCE,
    DYNAMIC_AVOIDANCE,
    ORIGIN_ALIGN,
    COMPANION_TRACK,
    ATTACK_ONLY,
    SWARM_FORMATION,
    HORIZONTAL_SQUARE,
    DYNAMIC_SQUARE,
    VERTICLE_CIRCLE,
    ATTACK_BALL
};
static const char* gui_ctrl_state_strings[] = { "USELESS",
                                                "START",
                                                "STOP",
                                                "TAKEOFF",
                                                "HOVER",
                                                "MOVE_POSITION",
                                                "LAND",
                                                "FOLLOW_GOAL",
                                                "STATIC_AVOIDANCE",
                                                "DYNAMIC_AVOIDANCE",
                                                "ORIGIN_ALIGN",
                                                "COMPANION_TRACK",
                                                "ATTACK_ONLY",
                                                "SWARM_FORMATION",
                                                "HORIZONTAL_SQUARE",
                                                "DYNAMIC_SQUARE",
                                                "VERTICLE_CIRCLE",
                                                "ATTACK_BALL" };
}  // namespace gui_ctrl_state

// } // namespace switch_ctrl_state

namespace firmware_type
{
enum FIRMWARE_TYPE
{
    PX4,
    BETAFLIGHT
};

}

#endif
