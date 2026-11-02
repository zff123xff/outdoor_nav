//
// Created by ljy on 12/17/23.
//
/************************************
 *
 * this is project to slove mqtt2ros and ros2mqtt
 * no Business Logic ,jusr a protocol conversion file !
 * Bussiness Logic is nx_backend project ->http://10.88.115.7:9001/luojiyuan/nx_backend.git
 *
 */

#ifndef MQTTSUBNODE_MQTTNODE_H
#define MQTTSUBNODE_MQTTNODE_H

#include <mqtt/async_client.h>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include "mqtt_utils.h"
#include "../mavlink_ros/uav_state.h"
#include "mqtt_response.h"
#include "mqtt/create_options.h"

class MqttNode : public virtual mqtt::callback,
                 public virtual mqtt::iaction_listener,
                 public rclcpp::Node {
public:
    using MessageHandler = std::function<void(const std::string &, const std::string &)>;
    using MessageHandlerWithProps = std::function<void(const std::string &, const std::string &,
                                                       const mqtt::properties &)>;

private:
    std::shared_ptr<mqtt::async_client> client_;
    std::string client_id_;
    std::map<std::string, MessageHandler> handlers_;

    rclcpp::QoS qos_sensor;
    rclcpp::QoS qos_default;
    rclcpp::QoS qos_profile;

    std::string curr_drone_id_;
    std::string prefix_;
    int vehicle_id_;
    double pub_rate_;
    std::string mqtt_address_;     // mqtt address
    std::string mqtt_username_;    // mqtt username
    std::string mqtt_passwd_;      // mqtt password
    int mqtt_timeout_;             // mqtt 超时时间
    int mqtt_keep_alive_interval_; // mqtt 心跳时间
    int mqtt_buffer_size_;         // mqtt 缓存区大小
    bool is_connected_ = false;
    bool is_summary_send_ = false;
    bool is_send_attack_imgs_ = false;
    float stream_code_rate_ = 1.0; // 控制第一视角视频流码率 默认为 1.0 Mbps

    /** mqtt 无人机消息汇总 **/
    RobotMajorState robot_major_state_; // 无人机主状态消息
    GlobalPosition global_position_;    // 无人机全局位置消息
    PX4State px4_state_;                // 无人机PX4状态消息
    UAVState uav_state_;                // 无人机汇总消息
    std::mutex robot_major_state_mutex_;
    std::mutex px4_state_mutex_;
    std::mutex global_position_mutex_;

    /** mqtt request/response **/
    std::map<std::string, MessageHandlerWithProps> handlers_with_props_;

    /** ros sub **/
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robotMajorState_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr globalPosition_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr statePX4_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr taskIndex_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr taskFeedback_sub_;
    rclcpp::Subscription<msg_set_msgs::msg::CoverIndex>::SharedPtr pastIndex_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr canConfirmedFlag_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr track_img_sub_;
    rclcpp::Subscription<msg_set_msgs::msg::TargetsPose>::SharedPtr targets_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr attack_confirm_sub_;

    /** ros sub topic **/
    std::string robotMajorState_sub_topic_;
    std::string globalPositionGps_sub_topic_;
    std::string statePX4_sub_topic_;
    std::string taskIndex_sub_topic_;
    std::string pastIndex_sub_topic_;
    std::string canConfirmedFlag_sub_topic_;
    std::string track_img_sub_topic_;
    std::string targets_sub_topic_;
    std::string attack_confirm_sub_topic_;

    /** ros pub **/
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ctrlStateGUI_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr armedGUI_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr takeOff_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr hover_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr land_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr return_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr streamFlag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr relayHeight_pub_;
    rclcpp::Publisher<msg_set_msgs::msg::MultiTask>::SharedPtr multiTask_pub_;
    rclcpp::Publisher<msg_set_msgs::msg::MultiTask>::SharedPtr taskReturn_pub_;
    rclcpp::Publisher<msg_set_msgs::msg::JumpTask>::SharedPtr jumpTask_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr taskPause_pub_;
    rclcpp::Publisher<msg_set_msgs::msg::CoverTask>::SharedPtr coverTask_pub_;
    rclcpp::Publisher<mavros_msgs::msg::WaypointList>::SharedPtr coverPath_pub_;
    rclcpp::Publisher<msg_set_msgs::msg::CoverParam>::SharedPtr coverParam_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr coverJump_pub_;
    rclcpp::Publisher<msg_set_msgs::msg::GlobalParam>::SharedPtr globalParam_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cancelTask_pub_;
    // rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr dronesAdjust_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr refGps_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr ptzControl_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr imageGet_pub_;
    rclcpp::Publisher<msg_set_msgs::msg::MultiGoal>::SharedPtr multiGoal_pub_;
    rclcpp::Publisher<msg_set_msgs::msg::AdjustTasks>::SharedPtr adjustTasks_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr lightControl_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr videoRecord_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shipRecord_pub_;

    /** ros pub topic **/
    std::string ctlStateDispatch_pub_topic_;
    std::string streamFlag_pub_topic_;
    std::string relayHeight_pub_topic_;
    std::string multiTask_pub_topic_;
    std::string taskReturn_pub_topic_;
    std::string jumpTask_pub_topic_;
    std::string taskPause_pub_topic_;
    std::string coverTask_pub_topic_;
    std::string coverPath_pub_topic_;
    std::string coverParam_pub_topic_;
    std::string coverJump_pub_topic_;
    std::string globalParam_pub_topic_;
    std::string cancelTask_pub_topic_;
    std::string dronesAdjust_pub_topic_;
    std::string refGps_pub_topic_;
    std::string ptzControl_pub_topic_;
    std::string imageGet_pub_topic_;
    std::string multiGoal_pub_topic_;
    std::string lightControl_pub_topic_;
    std::string videoRecord_pub_topic_;
    std::string shipRecord_pub_topic_;

    rclcpp::TimerBase::SharedPtr uav_state_output_timer_;

    rclcpp::TimerBase::SharedPtr mavros_state_monitor_timer_;
    rclcpp::Time recv_px4_state_backend_time_;
    rclcpp::Duration recv_px4_state_timeout_duration_;
    bool is_recv_px4_state_backend_;

    rclcpp::TimerBase::SharedPtr ctl_heartbeat_monitor_timer_;
    rclcpp::Time recv_ctl_heartbeat_time_;
    rclcpp::Duration recv_ctl_heartbeat_timeout_duration_;
    bool is_recv_ctl_heartbeat_;

public:
    MqttNode(const std::string &address, const std::string &client_id, const std::string &mqtt_username,
             const std::string mqtt_passwd, int mqtt_timeout, int mqtt_keep_alive_interval, int mqtt_buffer_size,
             const std::string &prefix, const std::string &curr_drone_id, int vehicle_id, double pub_rate,
             bool is_summary_send,
             std::string node_name, std::string node_namespace, bool intra_process_comms = false);

    MqttNode(const std::string &node_name, bool intra_process_comms = false);

    void uavStateOutputTimerCallback();

    void mavrosStateMonitorTimerCallback();

    void ctlHeartbeatMonitorTimerCallback();

    void mqttnode_init();

    /**
     * @brief Connects to the broker using the member client and options.
     */
    void connect();

    /**
     * @brief Callback for when the client has successfully connected to the
     * broker.
     *
     * Overrides mqtt::callback::connected(const std::string&).
     *
     * @param cause
     */
    void connected(const std::string &cause) override;

    /**
     * @brief Callback for when the client has lost connection to the broker.
     *
     * Overrides mqtt::callback::connection_lost(const std::string&).
     * @param cause
     */
    void connection_lost(const std::string &cause) override;

    /**
     * @brief Returns whether the client is connected to the broker.
     *
     * @return true if client is connected to the broker
     *
     */
    bool isConnected();

    /**
     * @brief Callback for when the client receives a MQTT message from the
     * broker. but ourself achieve how to message for callback.one topic corresponding to handler function
     * @param   mqtt_msg     MQTT message
     */
    void message_arrived(mqtt::const_message_ptr mqtt_msg) override;

    /**
     * @brief Callback for when delivery for a MQTT message has been completed.
     *
     * Overrides mqtt::callback::delivery_complete(mqtt::delivery_token_ptr).
     *
     * @param   token        token tracking the message delivery
     */
    void delivery_complete(mqtt::delivery_token_ptr token) override;

    /**
     * @brief Callback for when a MQTT action succeeds.
     *
     * Overrides mqtt::iaction_listener::on_success(const mqtt::token&).
     * Does nothing.
     *
     * @param   token        token tracking the action
     */
    void on_success(const mqtt::token &token) override;

    /**
     * @brief Callback for when a MQTT action fails.
     *
     * Overrides mqtt::iaction_listener::on_failure(const mqtt::token&).
     * Logs error.
     *
     * @param   token        token tracking the action
     */
    void on_failure(const mqtt::token &token) override;

    /**
     * @brief subcribe mqtttopic and callback to function
     * @param topic   mqtttopic
     * @param handler for mqtttopic callback function
     */
    void subscribe(const std::string &topic, MessageHandler handler);

    /**
     * @brief Subscribe with explicit QoS (e.g. thing/product services use QoS 1).
     */
    void subscribe(const std::string &topic, MessageHandler handler, int qos);

    /**
     * @brief Publish raw UTF-8 payload to MQTT (Thing API events/osd/services_reply).
     */
    bool publish_mqtt(const std::string &topic, const std::string &payload, int qos);

    /**
     * @brief subsribe mqtttopic and handle mqtt properties
     * @param topic mqtttopic
     * @param handler for mqtttopic callback function
     */
    void subscribeWithProps(const std::string &topic, MessageHandlerWithProps handler);

    /**
     * @brief unsubribe to mqtttopic
     * @param topic  mqtttopic
     */
    void unsubscribe(const std::string &topic);

    /** Remove handler map entry (e.g. when disconnected and unsubscribe() no-ops). */
    void remove_message_handler(const std::string &topic);

    /**
     *
     * @param msg rosmsg  funtion for callback
     */
    void statePX4Callback(const mavros_msgs::msg::State::SharedPtr msg);

    void robotMajorStateCallback(const std_msgs::msg::Int32::SharedPtr msg);

    void currGlobalPositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    void trackImgCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

    void targetsCallback(const msg_set_msgs::msg::TargetsPose::SharedPtr msg);

    void taskIndexCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void taskFeedbackCallback(const std_msgs::msg::String::SharedPtr msg);

    void pastIndexCallback(const msg_set_msgs::msg::CoverIndex::SharedPtr msg);

    void canConfirmedFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void attackConfirmCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     *
     * @param msg mqttmsg function for callback
     */
    void ctrStateCallback(const std::string &topic, const std::string &message);

    void takeOffCallback(const std::string &topic, const std::string &message);

    void hoverCallback(const std::string &topic, const std::string &message);

    void landCallback(const std::string &topic, const std::string &message);

    void returnCallback(const std::string &topic, const std::string &message);

    void armedCallback(const std::string &topic, const std::string &message);

    void attackImgsCtlCallback(const std::string &topic, const std::string &message);

    void streamFlagCallback(const std::string &topic, const std::string &message);

    void videoRecordCallback(const std::string &topic, const std::string &message);

    void shipRecordCallback(const std::string &topic, const std::string &message);

    void lightControlCallback(const std::string &topic, const std::string &message);

    void ctlHeartbeatCallback(const std::string &topic, const std::string &message);

    void relayCtlCallback(const std::string &topic, const std::string &message);

    void shipTaskCallback(const std::string &topic, const std::string &message);

    void taskStreamCallback(const std::string &topic, const std::string &message, const mqtt::properties &props);

    void taskReturnCallback(const std::string &topic, const std::string &message, const mqtt::properties &props);

    void taskStreamJumpCallback(const std::string &topic, const std::string &message, const mqtt::properties &props);

    void taskStreamPauseCallback(const std::string &topic, const std::string &message, const mqtt::properties &props);

    void
    followCoverAddTaskCallback(const std::string &topic, const std::string &message, const mqtt::properties &props);

    void
    followCoverParamAdjustCallback(const std::string &topic, const std::string &message, const mqtt::properties &props);

    void
    followCoverPathAdjustCallback(const std::string &topic, const std::string &message, const mqtt::properties &props);

    void
    followCoverTaskPauseCallback(const std::string &topic, const std::string &message, const mqtt::properties &props);

    void followCoverWaypointJumpCallback(const std::string &topic, const std::string &message,
                                         const mqtt::properties &props);

    void
    followCoverGlobalParamCallback(const std::string &topic, const std::string &message, const mqtt::properties &props);

    void
    followCoverTaskCancelCallback(const std::string &topic, const std::string &message, const mqtt::properties &props);

    void followCoverDroneGroupAdjustCallback(const std::string &topic, const std::string &message,
                                             const mqtt::properties &props);

    void refGpsCallback(const std::string &topic, const std::string &message, const mqtt::properties &props);

    void multiGoalCallback(const std::string &topic, const std::string &message, const mqtt::properties &props);

    void taskDronesAdjustCallback(const std::string &topic, const std::string &message, const mqtt::properties &props);

    void ptzControlCallback(const std::string &topic, const std::string &message);

    void imageGetCallback(const std::string &topic, const std::string &message);

    void subscribeTopics();

    bool
    checkMqttConditions(const std::string &request_topic, const std::string &message, const mqtt::properties &props,
                        std::string &response_topic, std::string &correlation_data);

    void
    mqttClientResponses(const std::string &response_topic, const std::string &correlation_data, MqttResponseCode code);

    void publishSystemVersion();

    ~MqttNode();
};

#endif // MQTTSUBNODE_MQTTNODE_H
