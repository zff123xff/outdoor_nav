//
// Created by libo on 2024/05/11.
//

#ifndef MQTT_TOPIC_H
#define MQTT_TOPIC_H

#include <string>

namespace MqttTopic
{
    std::string mqtt_uav_ctl_state = "/mqtt/state/ctrlStateGUI/";
    std::string mqtt_uav_armed = "/mqtt/armedGUI/";
    std::string mqtt_uav_takeoff = "/mqtt/takeOff/";
    std::string mqtt_uav_hover = "/mqtt/hover/";
    std::string mqtt_uav_land = "/mqtt/land/";
    std::string mqtt_uav_return = "/mqtt/return/";
    std::string mqtt_attack_imgs_ctl = "/mqtt/attack/imgsCtl/";
    std::string mqtt_stream_flag = "/mqtt/streamFlag/";
    std::string mqtt_ctl_heartbeat = "/mqtt/ctl/heartbeat";
    std::string mqtt_delay_ctl = "/mqtt/relayCtl/";
    std::string mqtt_targets = "/mqtt/targets/";
    std::string mqtt_ptz_control = "/mqtt/ptzControl/";
    std::string mqtt_image_get = "/mqtt/imageGet/";
    std::string mqtt_video_record = "/mqtt/videoRecord/";
    std::string mqtt_light_control = "/mqtt/lightControl/";
    std::string mqtt_ship_task = "/mqtt/web/task_stream/";
    std::string mqtt_ship_record = "/mqtt/ship/record/";

    std::string mqtt_request_multi_goal = "/mqtt/request/multiGoal/";
    std::string mqtt_request_task_deploy_task_stream = "/mqtt/request/task_deploy/nx/task_stream/";
    std::string mqtt_request_task_deploy_return = "/mqtt/request/task_deploy/nx/return/";
    std::string mqtt_request_task_deploy_stream_jump = "/mqtt/request/task_deploy/nx/stream_jump/";
    std::string mqtt_request_task_deploy_stream_pause = "/mqtt/request/task_deploy/nx/stream_pause/";
    std::string mqtt_request_follow_cover_add_task = "/mqtt/request/follow_cover/nx/add_task/";
    std::string mqtt_request_follow_cover_param_adjust = "/mqtt/request/follow_cover/nx/param_adjust/";
    std::string mqtt_request_follow_cover_path_adjust = "/mqtt/request/follow_cover/nx/path_adjust/";
    std::string mqtt_request_follow_cover_task_pause = "/mqtt/request/follow_cover/nx/task_pause/";
    std::string mqtt_request_follow_cover_waypoint_jump = "/mqtt/request/follow_cover/nx/waypoint_jump/";
    std::string mqtt_request_follow_cover_global_param = "/mqtt/request/follow_cover/nx/global_param/";
    std::string mqtt_request_follow_cover_task_cancel = "/mqtt/request/follow_cover/nx/task_cancel/";
    std::string mqtt_request_follow_cover_drone_group_adjust = "/mqtt/request/follow_cover/nx/drone_group_adjust/";
    std::string mqtt_request_ref_gps = "/mqtt/request/task_deploy/nx/ref_gps";
    std::string mqtt_request_task_drones_adjust = "/mqtt/request/task_deploy/nx/task_drones_adjust/";

    std::string mqtt_response_multi_goal = "/mqtt/response/multiGoal/";
    std::string mqtt_response_task_deploy_task_stream = "/mqtt/response/task_deploy/nx/task_stream/";
    std::string mqtt_response_task_deploy_return = "/mqtt/response/task_deploy/nx/return/";
    std::string mqtt_response_task_deploy_stream_jump = "/mqtt/response/task_deploy/nx/stream_jump/";
    std::string mqtt_response_task_deploy_stream_pause = "/mqtt/response/task_deploy/nx/stream_pause/";
    std::string mqtt_response_follow_cover_add_task = "/mqtt/response/follow_cover/nx/add_task/";
    std::string mqtt_response_follow_cover_param_adjust = "/mqtt/response/follow_cover/nx/param_adjust/";
    std::string mqtt_response_follow_cover_path_adjust = "/mqtt/response/follow_cover/nx/path_adjust/";
    std::string mqtt_response_follow_cover_task_pause = "/mqtt/response/follow_cover/nx/task_pause/";
    std::string mqtt_response_follow_cover_waypoint_jump = "/mqtt/response/follow_cover/nx/waypoint_jump/";
    std::string mqtt_response_follow_cover_global_param = "/mqtt/response/follow_cover/nx/global_param/";
    std::string mqtt_response_follow_cover_task_cancel = "/mqtt/response/follow_cover/nx/task_cancel/";
    std::string mqtt_response_follow_cover_drone_group_adjust = "/mqtt/response/follow_cover/nx/drone_group_adjust/";
    std::string mqtt_response_task_drones_adjust = "/mqtt/response/task_deploy/nx/task_drones_adjust/";

    std::string mqtt_robot_major_state = "/mqtt/robotMajorState/";
    std::string mqtt_px4_state = "/mqtt/px4State/";
    std::string mqtt_mavros_global_position = "/mqtt/mavros/global_position/global/";
    std::string mqtt_uav_state = "/mqtt/uav_state/";
    std::string mqtt_task_deploy_stream_feedback = "/mqtt/task_deploy/stream_feedback/";
    std::string mqtt_task_execution_feedback = "/mqtt/task_deploy/execution_feedback/";
    std::string mqtt_follow_cover_past_index = "/mqtt/follow_cover/pass_waypoint/";
    std::string mqtt_attack_imgs = "/mqtt/attack/images/";
    std::string mqtt_attack_imgs_status = "/mqtt/attack/imgs/status/";

    std::string mqtt_system_version = "/mqtt/system_version/";
    std::string mqtt_can_confirmed_flag = "/mqtt/task_attack/can_confirmed_flag/";
}

#endif // MQTT_TOPIC_H
