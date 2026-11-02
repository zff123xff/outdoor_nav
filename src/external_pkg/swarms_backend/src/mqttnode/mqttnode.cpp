//
// Created by ljy on 12/17/23.
//
#include <unordered_set>
#include "../include/mqttnode/mqttnode.hpp"
#include "../include/mqttnode/uav_ctl_state.h"
#include "../include/mqttnode/mqtt_topic.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

MqttNode::MqttNode(const std::string &address, const std::string &client_id, const std::string &mqtt_username,
                   const std::string mqtt_passwd, int mqtt_timeout, int mqtt_keep_alive_interval, int mqtt_buffer_size,
                   const std::string &prefix, const std::string &curr_drone_id, int vehicle_id, double pub_rate,
                   bool is_summary_send, std::string node_name = "mqttros_foxy_node", std::string node_namespace = "",
                   bool intra_process_comms)
        : mqtt_address_(address), client_id_(client_id), mqtt_username_(mqtt_username), mqtt_passwd_(mqtt_passwd),
          mqtt_timeout_(mqtt_timeout), mqtt_keep_alive_interval_(mqtt_keep_alive_interval),
          mqtt_buffer_size_(mqtt_buffer_size),
          prefix_(prefix), vehicle_id_(vehicle_id), curr_drone_id_(curr_drone_id), pub_rate_(pub_rate),
          is_summary_send_(is_summary_send),
          recv_px4_state_timeout_duration_(2s), recv_ctl_heartbeat_timeout_duration_(5s), robot_major_state_(),
          global_position_(), px4_state_(),
          Node(node_name, node_namespace, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
          qos_default(intra_process_comms
                      ? rclcpp::QoS(
                          rclcpp::QoSInitialization(
                                  rmw_qos_profile_sensor_data.history,
                                  10),
                          rmw_qos_profile_default)
                      : rclcpp::SystemDefaultsQoS()),
          qos_sensor(rclcpp::QoS(
                  rclcpp::QoSInitialization(
                          rmw_qos_profile_sensor_data.history,
                          10),
                  rmw_qos_profile_sensor_data)),
          qos_profile(intra_process_comms
                      ? rclcpp::QoS(
                          rclcpp::KeepLast(10))
                              .reliability(
                                      RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                              .durability(
                                      RMW_QOS_POLICY_DURABILITY_VOLATILE)
                      : rclcpp::QoS(
                          rclcpp::KeepLast(10))
                              .reliability(
                                      RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                              .durability(
                                      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)) {

    mqttnode_init();
}

MqttNode::MqttNode(const std::string &node_name, bool intra_process_comms) : Node(node_name, "",
                                                                                  rclcpp::NodeOptions().use_intra_process_comms(
                                                                                          intra_process_comms)),
                                                                             recv_px4_state_timeout_duration_(2s),
                                                                             recv_ctl_heartbeat_timeout_duration_(5s),
                                                                             qos_default(intra_process_comms
                                                                                         ? rclcpp::QoS(
                                                                                             rclcpp::QoSInitialization(
                                                                                                     rmw_qos_profile_sensor_data.history,
                                                                                                     10),
                                                                                             rmw_qos_profile_default)
                                                                                         : rclcpp::SystemDefaultsQoS()),
                                                                             qos_sensor(rclcpp::QoS(
                                                                                     rclcpp::QoSInitialization(
                                                                                             rmw_qos_profile_sensor_data.history,
                                                                                             10),
                                                                                     rmw_qos_profile_sensor_data)),
                                                                             qos_profile(intra_process_comms
                                                                                         ? rclcpp::QoS(
                                                                                             rclcpp::KeepLast(10))
                                                                                                 .reliability(
                                                                                                         RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                                                                                 .durability(
                                                                                                         RMW_QOS_POLICY_DURABILITY_VOLATILE)
                                                                                         : rclcpp::QoS(
                                                                                             rclcpp::KeepLast(10))
                                                                                                 .reliability(
                                                                                                         RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                                                                                 .durability(
                                                                                                         RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)) {

    prefix_ = this->declare_parameter("prefix", "drone_");
    vehicle_id_ = this->declare_parameter("vehicle_id", 0);
    pub_rate_ = this->declare_parameter("pub_rate", 5.0);
    curr_drone_id_ = this->declare_parameter("curr_drone_id", "");
    mqtt_address_ = this->declare_parameter("address", "tcp://127.0.0.1:1883");
    mqtt_username_ = this->declare_parameter("mqtt_username", "admin");
    mqtt_passwd_ = this->declare_parameter("mqtt_passwd", "nanhu@2020");
    mqtt_timeout_ = this->declare_parameter("mqtt_timeout", 60);
    mqtt_keep_alive_interval_ = this->declare_parameter("mqtt_keep_alive_interval", 100);
    mqtt_buffer_size_ = this->declare_parameter("mqtt_buffer_size", 62914560); // 60M
    is_summary_send_ = this->declare_parameter("is_summary_send", true);

    auto time = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
    time_t timestamp = time.time_since_epoch().count();
    client_id_ = std::to_string(timestamp) + "pubsub" + std::to_string(vehicle_id_);

    mqttnode_init();
}

void MqttNode::mqttnode_init() {

    std::string msg_set_msgs_path = ament_index_cpp::get_package_share_directory("msg_set_msgs");
    if (msg_set_msgs_path == "") {
        RCLCPP_ERROR_STREAM(get_logger(), "Can not find msg_set_msgs, exit...");
        exit(EXIT_FAILURE);
    }

    YAML::Node topic_config;
    try {
        std::string topic_config_path = msg_set_msgs_path + "/config/topic_key.yaml";
        topic_config = YAML::LoadFile(topic_config_path);
    }
    catch (const YAML::ParserException) {
        RCLCPP_ERROR_STREAM(get_logger(), "YAML FILE 'topic_key' parse error, exit...");
        exit(EXIT_FAILURE);
    }
    catch (const YAML::BadFile) {
        RCLCPP_ERROR_STREAM(get_logger(), "YAML FILE 'topic_key' cannot be loaded, exit...");
        exit(EXIT_FAILURE);
    }

    try {
        robotMajorState_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["uav_state_machine"]["usm_state_topic"].as<std::string>() +
                "/backend";
        globalPositionGps_sub_topic_ = curr_drone_id_ + "/mavros/global_position/global" + "/backend";
        statePX4_sub_topic_ = curr_drone_id_ + "/mavros/state" + "/backend";
        taskIndex_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["task"]["task_manage"]["task_index_topic"].as<std::string>() +
                "/backend";
        pastIndex_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["task"]["task_manage"]["cover_reach_index"].as<std::string>() +
                "/backend";
        canConfirmedFlag_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["task"]["task_manage"]["can_confirmed_flag"].as<std::string>() +
                "/backend";
        track_img_sub_topic_ = curr_drone_id_ + "/" + topic_config["perception"]["track_img_topic"].as<std::string>();
        targets_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["task"]["task_attack"]["target_info_topic"].as<std::string>() +
                "/backend";
        attack_confirm_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["task"]["task_fixedpoint"]["operation_confirm"].as<std::string>() +
                "/backend";

        ctlStateDispatch_pub_topic_ =
                curr_drone_id_ + "/" + topic_config["backend"]["ctl_state_dispatch"].as<std::string>();
        lightControl_pub_topic_ = curr_drone_id_ + "/" + topic_config["light"]["light_ctrl_topic"].as<std::string>();
        streamFlag_pub_topic_ =
                curr_drone_id_ + "/" + topic_config["perception"]["stream_flag_topic"].as<std::string>();
        videoRecord_pub_topic_ =
                curr_drone_id_ + "/" + topic_config["perception"]["video_record_ctrl_topic"].as<std::string>();
        relayHeight_pub_topic_ = curr_drone_id_ + "/" + topic_config["backend"]["relay_height"].as<std::string>();
        multiTask_pub_topic_ = curr_drone_id_ + "/" + topic_config["backend"]["multitask"].as<std::string>();
        taskReturn_pub_topic_ = curr_drone_id_ + "/" + topic_config["backend"]["return_task"].as<std::string>();
        jumpTask_pub_topic_ = curr_drone_id_ + "/" + topic_config["backend"]["jump_task"].as<std::string>();
        taskPause_pub_topic_ = curr_drone_id_ + "/" + topic_config["backend"]["pause_task"].as<std::string>();
        coverTask_pub_topic_ = curr_drone_id_ + "/" + topic_config["backend"]["cover_task"].as<std::string>();
        coverPath_pub_topic_ = curr_drone_id_ + "/" + topic_config["backend"]["cover_path"].as<std::string>();
        coverParam_pub_topic_ = curr_drone_id_ + "/" + topic_config["backend"]["cover_param"].as<std::string>();
        coverJump_pub_topic_ = curr_drone_id_ + "/" + topic_config["backend"]["cover_goalpoint"].as<std::string>();
        globalParam_pub_topic_ = curr_drone_id_ + "/" + topic_config["backend"]["global_param"].as<std::string>();
        cancelTask_pub_topic_ = curr_drone_id_ + "/" + topic_config["backend"]["cancel_task"].as<std::string>();
        dronesAdjust_pub_topic_ = curr_drone_id_ + "/" + topic_config["backend"]["drones_adjust"].as<std::string>();
        ptzControl_pub_topic_ = curr_drone_id_ + "/" + topic_config["backend"]["ptz_control"].as<std::string>();
        imageGet_pub_topic_ = curr_drone_id_ + "/" + topic_config["backend"]["image_get"].as<std::string>();

        shipRecord_pub_topic_ = curr_drone_id_ + "/swarms_backend/pose_record";

        multiGoal_pub_topic_ =
                curr_drone_id_ + "/" + topic_config["algo"]["MOVE_POSITION"]["input_topic"].as<std::string>();

        refGps_pub_topic_ = "/ui_to_mqtt_ref_gps";
    }
    catch (const YAML::Exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "YAML FILE 'topic_key' find key error: " << std::string(e.what()) << ", exit...");
        exit(EXIT_FAILURE);
    }

    // ROS subscriber
    //    statePX4_sub_ = this->create_subscription<mavros_msgs::msg::State>(statePX4_sub_topic_, qos_sensor,
    //                                                                       std::bind(&MqttNode::statePX4Callback, this,
    //                                                                                 _1));
    //
    //    robotMajorState_sub_ = this->create_subscription<std_msgs::msg::Int32>(robotMajorState_sub_topic_, qos_sensor,
    //                                                                           std::bind(&MqttNode::robotMajorStateCallback,
    //                                                                                     this, _1));
    //
    //    globalPosition_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(globalPositionGps_sub_topic_,
    //                                                                                 qos_sensor, std::bind(
    //                    &MqttNode::currGlobalPositionCallback, this, _1));
    //
    track_img_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(track_img_sub_topic_, qos_sensor,
                                                                               std::bind(&MqttNode::trackImgCallback,
                                                                                         this, _1));
    taskFeedback_sub_ = this->create_subscription<std_msgs::msg::String>(
            curr_drone_id_ + "/task_feedback", qos_default,
            std::bind(&MqttNode::taskFeedbackCallback, this, _1));
    //
    //    targets_sub_ = this->create_subscription<msg_set_msgs::msg::TargetsPose>(targets_sub_topic_, qos_sensor,
    //                                                                             std::bind(&MqttNode::targetsCallback, this,
    //                                                                                       _1));
    //
    //    // task state
    //    taskIndex_sub_ = this->create_subscription<std_msgs::msg::UInt8>(taskIndex_sub_topic_, qos_sensor,
    //                                                                     std::bind(&MqttNode::taskIndexCallback, this, _1));
    //
    //    pastIndex_sub_ = this->create_subscription<msg_set_msgs::msg::CoverIndex>(pastIndex_sub_topic_, qos_sensor,
    //                                                                              std::bind(&MqttNode::pastIndexCallback,
    //                                                                                        this, _1));
    //
    //    canConfirmedFlag_sub_ = this->create_subscription<std_msgs::msg::Bool>(canConfirmedFlag_sub_topic_, qos_sensor,
    //                                                                           std::bind(
    //                                                                                   &MqttNode::canConfirmedFlagCallback,
    //                                                                                   this, _1));
    //
    //    attack_confirm_sub_ = this->create_subscription<std_msgs::msg::Bool>(attack_confirm_sub_topic_, qos_sensor,
    //                                                                         std::bind(&MqttNode::attackConfirmCallback,
    //                                                                                   this, _1));

    // ROS publisher
    // uav ctl
    ctrlStateGUI_pub_ = this->create_publisher<std_msgs::msg::Int32>(ctlStateDispatch_pub_topic_, qos_default);

    takeOff_pub_ = this->create_publisher<std_msgs::msg::Int32>(ctlStateDispatch_pub_topic_, qos_default);

    hover_pub_ = this->create_publisher<std_msgs::msg::Int32>(ctlStateDispatch_pub_topic_, qos_default);

    land_pub_ = this->create_publisher<std_msgs::msg::Int32>(ctlStateDispatch_pub_topic_, qos_default);

    return_pub_ = this->create_publisher<std_msgs::msg::Int32>(ctlStateDispatch_pub_topic_, qos_default);

    armedGUI_pub_ = this->create_publisher<std_msgs::msg::Int32>(ctlStateDispatch_pub_topic_, qos_default);

    streamFlag_pub_ = this->create_publisher<std_msgs::msg::Float32>(streamFlag_pub_topic_, qos_default);

    relayHeight_pub_ = this->create_publisher<std_msgs::msg::Float32>(relayHeight_pub_topic_, qos_default);

    // task ctl
    multiTask_pub_ = this->create_publisher<msg_set_msgs::msg::MultiTask>(multiTask_pub_topic_, qos_profile);

    taskReturn_pub_ = this->create_publisher<msg_set_msgs::msg::MultiTask>(taskReturn_pub_topic_, qos_default);

    jumpTask_pub_ = this->create_publisher<msg_set_msgs::msg::JumpTask>(jumpTask_pub_topic_, qos_default);

    taskPause_pub_ = this->create_publisher<std_msgs::msg::Bool>(taskPause_pub_topic_, qos_default);

    coverTask_pub_ = this->create_publisher<msg_set_msgs::msg::CoverTask>(coverTask_pub_topic_, qos_profile);

    coverPath_pub_ = this->create_publisher<mavros_msgs::msg::WaypointList>(coverPath_pub_topic_, qos_default);

    coverParam_pub_ = this->create_publisher<msg_set_msgs::msg::CoverParam>(coverParam_pub_topic_, qos_default);

    coverJump_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(coverJump_pub_topic_, qos_default);

    globalParam_pub_ = this->create_publisher<msg_set_msgs::msg::GlobalParam>(globalParam_pub_topic_, qos_default);

    cancelTask_pub_ = this->create_publisher<std_msgs::msg::Bool>(cancelTask_pub_topic_, qos_default);

    // dronesAdjust_pub_ = this->create_publisher<std_msgs::msg::Int8MultiArray>(dronesAdjust_pub_topic_, qos_default);

    adjustTasks_pub_ = this->create_publisher<msg_set_msgs::msg::AdjustTasks>(dronesAdjust_pub_topic_, qos_default);

    refGps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(refGps_pub_topic_, qos_default);

    // ptz
    ptzControl_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(ptzControl_pub_topic_, qos_default);
    // save img
    imageGet_pub_ = this->create_publisher<std_msgs::msg::UInt8>(imageGet_pub_topic_, qos_default);

    videoRecord_pub_ = this->create_publisher<std_msgs::msg::Int8>(videoRecord_pub_topic_, qos_default);

    lightControl_pub_ = this->create_publisher<std_msgs::msg::Int8>(lightControl_pub_topic_, qos_default);

    // 多点发布
    multiGoal_pub_ = this->create_publisher<msg_set_msgs::msg::MultiGoal>(multiGoal_pub_topic_, qos_default);

    shipRecord_pub_ = this->create_publisher<std_msgs::msg::Bool>(shipRecord_pub_topic_, qos_default);

    //    mavros_state_monitor_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
    //                                                          std::bind(&MqttNode::mavrosStateMonitorTimerCallback, this));
    //    recv_px4_state_backend_time_ = this->now();
    //    is_recv_px4_state_backend_ = false;
    //
    //    ctl_heartbeat_monitor_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
    //                                                           std::bind(&MqttNode::ctlHeartbeatMonitorTimerCallback,
    //                                                                     this));
    //    recv_ctl_heartbeat_time_ = this->now();
    //    is_recv_ctl_heartbeat_ = false;
    //
    //    if (is_summary_send_) {
    //        RCLCPP_INFO_STREAM(get_logger(), "uav state summary send enable...");
    //        uav_state_output_timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
    //                                                          std::bind(&MqttNode::uavStateOutputTimerCallback, this));
    //    }

    // MQTT v5.0 client
    // mqtt::create_options createOpts(MQTTVERSION_5);
    const auto opts = mqtt::create_options_builder()
            .send_while_disconnected(false, false)
            .max_buffered_messages(100)
            .delete_oldest_messages()
            .mqtt_version(MQTTVERSION_5)
            .finalize();

    client_ = std::make_shared<mqtt::async_client>(mqtt_address_, client_id_, opts);

    // setup MQTT callbacks
    client_->set_callback(*this);

}

void MqttNode::publishSystemVersion() {
    std::string exec_path = FileUtils::getExecutablePath();
    std::string system_version_path = exec_path + "/../../../../../system_version.txt";
    std::cout << "system_version_path: " << system_version_path << std::endl;
    std::ifstream file(system_version_path);
    if (file.is_open()) {
        std::string version = "";
        std::getline(file, version);
        if (version != "") {
            if (!is_connected_) {
                RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot pub system version.");
                return;
            }
            try {
                std::string jsonData = JsonUtils::jsonValue(version);
                auto pubmsg = mqtt::make_message(MqttTopic::mqtt_system_version + std::to_string(vehicle_id_),
                                                 jsonData);
                pubmsg->set_qos(0);
                client_->publish(pubmsg);
            }
            catch (const std::exception &e) {
                RCLCPP_ERROR_STREAM(get_logger(), "Error in publishSystemVersion: " << std::string(e.what()));
            }
            catch (const mqtt::exception &e) {
                RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
            }
        }
    } else {
        RCLCPP_ERROR_STREAM(get_logger(), "Can not find system_version.txt ...");
    }
    file.close();
}

void MqttNode::uavStateOutputTimerCallback() {
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot pub uav state.");
        return;
    }
    bool shouldSend = false;
    {
        std::lock_guard<std::mutex> lock(robot_major_state_mutex_);
        uav_state_.robotMajorState_ = robot_major_state_;
        if (robot_major_state_.isUpdated) {
            robot_major_state_.isUpdated = false;
            shouldSend = true;
        }
    }
    {
        std::lock_guard<std::mutex> lock(global_position_mutex_);
        uav_state_.globalPosition_ = global_position_;
        if (global_position_.isUpdated) {
            global_position_.isUpdated = false;
            shouldSend = true;
        }
    }
    {
        std::lock_guard<std::mutex> lock(px4_state_mutex_);
        uav_state_.px4State_ = px4_state_;
        if (px4_state_.isUpdated) {
            RCLCPP_INFO_STREAM(get_logger(), "send uav state include px4_state.");
            px4_state_.isUpdated = false;
            shouldSend = true;
        }
    }

    if (shouldSend) {
        try {
            std::string jsonData = JsonUtils::jsonValue(uav_state_);
            RCLCPP_INFO_STREAM(get_logger(), "uavStateOutputTimerCallback: size of jsonData is  " << jsonData.size());
            auto pubmsg = mqtt::make_message(MqttTopic::mqtt_uav_state + std::to_string(vehicle_id_), jsonData);
            pubmsg->set_qos(0);
            client_->publish(pubmsg);
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(get_logger(), "Error in uavStateOutputTimerCallback: " << std::string(e.what()));
        }
        catch (const mqtt::exception &e) {
            RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
        }
    }
}

void MqttNode::mavrosStateMonitorTimerCallback() {
    auto now = this->now();
    if ((now - recv_px4_state_backend_time_) > recv_px4_state_timeout_duration_) {
        RCLCPP_WARN_STREAM(get_logger(), "mqttnode px4 state not recv!");
        is_recv_px4_state_backend_ = false;
    } else {
        if (is_recv_px4_state_backend_ == false) {
            const std::string GREEN = "\033[32m"; // green
            const std::string RESET = "\033[0m";  // reset to default color
            RCLCPP_INFO_STREAM(get_logger(), GREEN << "mqttnode px4 state recv!" << RESET);
        }
        is_recv_px4_state_backend_ = true;
    }
    // if (is_connected_) {
    //     int attack_imgs_output_status = is_send_attack_imgs_ ? 1 : 0;
    //     std::string jsonData = JsonUtils::jsonValue(attack_imgs_output_status);
    //     auto pubmsg = mqtt::make_message(MqttTopic::mqtt_attack_imgs_status + std::to_string(vehicle_id_), jsonData);
    //     pubmsg->set_qos(0);
    //     client_->publish(pubmsg);
    // }
}

void MqttNode::ctlHeartbeatMonitorTimerCallback() {
    auto now = this->now();
    if ((now - recv_ctl_heartbeat_time_) > recv_ctl_heartbeat_timeout_duration_) {
        RCLCPP_WARN_STREAM(get_logger(), "ctl heartbeat not recv!");
        is_recv_ctl_heartbeat_ = false;
    } else {
        if (is_recv_ctl_heartbeat_ == false) {
            const std::string GREEN = "\033[32m"; // green
            const std::string RESET = "\033[0m";  // reset to default color
            RCLCPP_INFO_STREAM(get_logger(), GREEN << "ctl heartbeat recv!" << RESET);
        }
        is_recv_ctl_heartbeat_ = true;
    }
}

void MqttNode::connect() {
    // MQTT v5.0 client connect
    auto connOpts = mqtt::connect_options_builder()
            .mqtt_version(MQTTVERSION_5)
            .properties({{mqtt::property::SESSION_EXPIRY_INTERVAL, 604800}})
            .automatic_reconnect(true)
            .max_inflight(mqtt_buffer_size_)
            .clean_session(false) // 保持会话以利用QoS功能
            .user_name(mqtt_username_)
            .password(mqtt_passwd_)
            .keep_alive_interval(std::chrono::seconds(
                    mqtt_keep_alive_interval_))                       // 客户端和服务器端的保活间隔（心跳时间）,服务器在 Keep Alive 时长的 1.5 倍时间内没有收到来自客户端的 MQTT 控制报文，它必须关闭与客户端的网络连接
            .connect_timeout(std::chrono::seconds(mqtt_timeout_)) // 连接超时时间设置
            .finalize();
    try {
        client_->connect(connOpts, nullptr, *this);
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Connection to broker failed: " << std::string(e.what()));
        exit(EXIT_FAILURE);
    }
}

void MqttNode::connected(const std::string &cause) {
    is_connected_ = true;
    RCLCPP_INFO_STREAM(get_logger(), "Connected to broker successful");
    subscribeTopics();
    publishSystemVersion();
}

void MqttNode::connection_lost(const std::string &cause) {
    RCLCPP_WARN_STREAM(get_logger(), "Connection to broker lost, will try to reconnect...");
    is_connected_ = false;
    connect();
}

bool MqttNode::isConnected() {
    return is_connected_;
}

void MqttNode::message_arrived(mqtt::const_message_ptr mqtt_msg) {
    std::string topic = mqtt_msg->get_topic();   // 话题
    std::string message = mqtt_msg->to_string(); // 消息
    auto props = mqtt_msg->get_properties();     // 消息属性

    // 首先尝试使用包含属性的处理函数
    auto handler_with_props_it = handlers_with_props_.find(topic);
    if (handler_with_props_it != handlers_with_props_.end()) {
        // 调用找到的处理函数，包括属性
        handler_with_props_it->second(topic, message, props);
        return;
    }

    // 查找处理函数
    auto handler_it = handlers_.find(topic);
    if (handler_it != handlers_.end()) {
        // 调用找到的处理函数
        handler_it->second(topic, message);
    } else {
        // 如果没有找到处理函数，可以记录警告或者忽略
        RCLCPP_WARN_STREAM(get_logger(), "No handler registered for topic: " << topic);
    }
}

void MqttNode::delivery_complete(mqtt::delivery_token_ptr token) {}

void MqttNode::on_success(const mqtt::token &token) {
    is_connected_ = true;
}

void MqttNode::on_failure(const mqtt::token &token) {
    RCLCPP_ERROR_STREAM(get_logger(),
                        "Connection to broker failed (return code " << std::to_string(token.get_return_code())
                                                                    << "  ), will automatically retry...");
    is_connected_ = false;
}

void MqttNode::subscribe(const std::string &topic, MessageHandler handler) {
    subscribe(topic, handler, 2);
}

void MqttNode::subscribe(const std::string &topic, MessageHandler handler, int qos) {
    if (topic.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null topic");
        return;
    }
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        handlers_[topic] = handler;
        mqtt::subscribe_options opts;
        opts.set_no_local(true);
        client_->subscribe(topic, qos, opts);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in subscibe " << std::string(e.what()));
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
    }
}

bool MqttNode::publish_mqtt(const std::string &topic, const std::string &payload, int qos) {
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot publish.");
        return false;
    }
    try {
        auto msg = mqtt::make_message(topic, payload);
        msg->set_qos(qos);
        client_->publish(msg);
        return true;
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "publish_mqtt error: " << std::string(e.what()));
        return false;
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "publish_mqtt mqtt exception: " << e.what());
        return false;
    }
}

void MqttNode::subscribeWithProps(const std::string &topic, MessageHandlerWithProps handler) {
    if (topic.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null topic");
        return;
    }

    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }

    try {
        // 存储带有属性的处理函数
        handlers_with_props_[topic] = handler;
        // 设置订阅选项，例如 No Local flag，以防止接收自己发布的消息
        mqtt::subscribe_options opts;
        opts.set_no_local(true);
        // 创建一个空的 MQTT v5 属性列表
        mqtt::properties props;
        // 为指定主题订阅消息，使用QOS=2
        client_->subscribe(topic, 2, opts, props);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in subscibe " << std::string(e.what()));
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
    }
}

void MqttNode::remove_message_handler(const std::string &topic) {
    handlers_.erase(topic);
}

void MqttNode::unsubscribe(const std::string &topic) {
    // 检查消息是否为空
    if (topic.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null topic");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        remove_message_handler(topic);
        return;
    }
    try {
        client_->unsubscribe(topic);
        handlers_.erase(topic);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in unsubscibe " << std::string(e.what()));
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
    }
}

// ------------------------ ROS callback: recv ROS, and send with MQTT --------------------------------

void MqttNode::robotMajorStateCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in robotMajorState.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        if (!is_summary_send_) {
            int data = msg->data;
            std::string jsonData = JsonUtils::jsonValue(data);
            auto pubmsg = mqtt::make_message(MqttTopic::mqtt_robot_major_state + std::to_string(vehicle_id_), jsonData);
            pubmsg->set_qos(0);
            client_->publish(pubmsg);
        } else {
            std::lock_guard<std::mutex> lock(robot_major_state_mutex_);
            robot_major_state_.state = msg->data;
            robot_major_state_.isUpdated = true;
        }
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in robotMajorStateCallback: " << std::string(e.what()));
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
    }
}

void MqttNode::statePX4Callback(const mavros_msgs::msg::State::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in statePX4.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        recv_px4_state_backend_time_ = this->now();
        if (!is_summary_send_) {
            std::string jsonData = JsonUtils::jsonValue(msg);
            auto pubmsg = mqtt::make_message(MqttTopic::mqtt_px4_state + std::to_string(vehicle_id_), jsonData);
            pubmsg->set_qos(0);
            client_->publish(pubmsg);
        } else {
            std::lock_guard<std::mutex> lock(px4_state_mutex_);
            px4_state_.connected = msg->connected;
            px4_state_.armed = msg->armed;
            px4_state_.guided = msg->guided;
            px4_state_.manual_input = msg->manual_input;
            px4_state_.mode = msg->mode;
            px4_state_.isUpdated = true;
        }
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in statePX4Callback: " << std::string(e.what()));
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
    }
}

void MqttNode::currGlobalPositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in currGlobalPosition.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        if (!is_summary_send_) {
            std::string jsonData = JsonUtils::jsonValue(msg);
            auto pubmsg = mqtt::make_message(MqttTopic::mqtt_mavros_global_position + std::to_string(vehicle_id_),
                                             jsonData);
            pubmsg->set_qos(0);
            client_->publish(pubmsg);
        } else {
            std::lock_guard<std::mutex> lock(global_position_mutex_);
            global_position_.status = msg->status.status;
            global_position_.service = msg->status.service;
            global_position_.latitude = msg->latitude;
            global_position_.longitude = msg->longitude;
            global_position_.altitude = msg->altitude;
            global_position_.isUpdated = true;
        }
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in currGlobalPositionCallback: " << std::string(e.what()));
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
    }
}

void MqttNode::trackImgCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in trackImgCallback.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        if (!is_send_attack_imgs_) {
            return;
        }
        std::string base64_str = ImgUtils::bufferToBase64(msg->data.data(), msg->data.size());
        RCLCPP_INFO_STREAM(get_logger(), "track img size: " << base64_str.length());
        auto pubmsg = mqtt::make_message(MqttTopic::mqtt_attack_imgs + std::to_string(vehicle_id_), base64_str);
        pubmsg->set_qos(0);
        client_->publish(pubmsg);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in trackImgCallback: " << std::string(e.what()));
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
    }
}

void MqttNode::targetsCallback(const msg_set_msgs::msg::TargetsPose::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in targetsCallback.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        std::string jsonData = JsonUtils::jsonValue(msg);
        auto pubmsg = mqtt::make_message(MqttTopic::mqtt_targets + std::to_string(vehicle_id_), jsonData);
        pubmsg->set_qos(0);
        client_->publish(pubmsg);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in targetsCallback: " << std::string(e.what()));
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
    }
}

void MqttNode::taskIndexCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in taskIndexCallback.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        std::string jsonData = JsonUtils::jsonValue(msg);
        auto pubmsg = mqtt::make_message(MqttTopic::mqtt_task_deploy_stream_feedback + std::to_string(vehicle_id_),
                                         jsonData);
        pubmsg->set_qos(0);
        client_->publish(pubmsg);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in taskIndexCallback: " << std::string(e.what()));
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
    }
}

void MqttNode::taskFeedbackCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null task execution feedback message.");
        return;
    }
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot publish task feedback.");
        return;
    }
    try {
        auto pubmsg = mqtt::make_message(
                MqttTopic::mqtt_task_execution_feedback + std::to_string(vehicle_id_), msg->data);
        pubmsg->set_qos(1);
        client_->publish(pubmsg);
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in taskFeedbackCallback: " << e.what());
    } catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "MQTT exception in taskFeedbackCallback: " << e.what());
    }
}

void MqttNode::pastIndexCallback(const msg_set_msgs::msg::CoverIndex::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in pastIndexCallback.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        std::string jsonData = JsonUtils::jsonValue(msg);
        auto pubmsg = mqtt::make_message(MqttTopic::mqtt_follow_cover_past_index + std::to_string(vehicle_id_),
                                         jsonData);
        pubmsg->set_qos(0);
        client_->publish(pubmsg);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in pastIndexCallback: " << std::string(e.what()));
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
    }
}

void MqttNode::canConfirmedFlagCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in canConfirmedFlagCallback.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        std::string jsonData = JsonUtils::jsonValue(msg);
        auto pubmsg = mqtt::make_message(MqttTopic::mqtt_can_confirmed_flag + std::to_string(vehicle_id_), jsonData);
        pubmsg->set_qos(0);
        client_->publish(pubmsg);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in canConfirmedFlagCallback: " << std::string(e.what()));
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
    }
}

void MqttNode::attackConfirmCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in attackConfirmCallback.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        std::string jsonData = JsonUtils::jsonValue(msg);
        auto pubmsg = mqtt::make_message(MqttTopic::mqtt_can_confirmed_flag + std::to_string(vehicle_id_), jsonData);
        pubmsg->set_qos(0);
        client_->publish(pubmsg);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in attackConfirmCallback: " << std::string(e.what()));
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
    }
}

// ------------------------ ROS callback: recv ROS, and send with MQTT --------------------------------

// ------------------------ MQTT callback: recv MQTT, and send with ROS --------------------------------

void MqttNode::ctrStateCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in ctrState.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        int mqtt_msg = std::make_unique<JsonUtils>()->msgIntValue(message);
        //        std::unordered_set<int> valid_common_ctl = {CtlState::FollowCancel, CtlState::AttackGoal, CtlState::AlgoPointAttack};
        //        if(valid_common_ctl.find(mqtt_msg) == valid_common_ctl.end()) {
        //            RCLCPP_WARN_STREAM(get_logger(), "ctrStateCallback: recv error input");
        //            return;
        //        }
        std_msgs::msg::Int32 pub_msg;
        pub_msg.data = mqtt_msg;
        ctrlStateGUI_pub_->publish(pub_msg);
        RCLCPP_INFO_STREAM(get_logger(), "Send ros_message  value is = " + std::to_string(pub_msg.data) + " to " +
                                         ctrlStateGUI_pub_->get_topic_name() + " success");
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in ctrStateCallback: " << std::string(e.what()));
    }
}

void MqttNode::armedCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in armed.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        int mqtt_msg = std::make_unique<JsonUtils>()->msgIntValue(message);
        std_msgs::msg::Int32 ros_message;
        if (mqtt_msg == 1) {
            ros_message.data = CtlState::Armed; // 解锁
        } else if (mqtt_msg == -1) {
            ros_message.data = CtlState::DisArmed; // 上锁
        } else {
            RCLCPP_WARN_STREAM(get_logger(), "armedCallback: recv error input");
            return;
        }
        armedGUI_pub_->publish(ros_message);
        RCLCPP_INFO_STREAM(get_logger(), "Send ros_message  value is = " + std::to_string(ros_message.data) + " to " +
                                         armedGUI_pub_->get_topic_name() + " success");
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in armedCallback: " << std::string(e.what()));
    }
}

void MqttNode::hoverCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in hover.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        int mqtt_msg = std::make_unique<JsonUtils>()->msgIntValue(message);
        if (mqtt_msg != CtlState::Hover) {
            RCLCPP_WARN_STREAM(get_logger(), "hoverCallback: recv error input");
            return;
        }
        std_msgs::msg::Int32 ros_message;
        ros_message.data = mqtt_msg;
        hover_pub_->publish(ros_message);
        RCLCPP_INFO_STREAM(get_logger(), "Send ros_message  value is = " + std::to_string(ros_message.data) + " to " +
                                         hover_pub_->get_topic_name() + " success");
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in hoverCallback: " << std::string(e.what()));
    }
}

void MqttNode::takeOffCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in takeOff.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        int mqtt_msg = std::make_unique<JsonUtils>()->msgIntValue(message);
        if (mqtt_msg != CtlState::TakeOff) {
            RCLCPP_WARN_STREAM(get_logger(), "takeOffCallback: recv error input");
            return;
        }
        std_msgs::msg::Int32 ros_message;
        ros_message.data = mqtt_msg;
        takeOff_pub_->publish(ros_message);
        RCLCPP_INFO_STREAM(get_logger(), "Send ros_message  value is = " + std::to_string(ros_message.data) + " to " +
                                         takeOff_pub_->get_topic_name() + " success");
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in takeOffCallback: " << std::string(e.what()));
    }
}

void MqttNode::landCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in land.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        int mqtt_msg = std::make_unique<JsonUtils>()->msgIntValue(message);
        if (mqtt_msg != CtlState::Land) {
            RCLCPP_WARN_STREAM(get_logger(), "landCallback: recv error input");
            return;
        }
        std_msgs::msg::Int32 ros_message;
        ros_message.data = mqtt_msg;
        land_pub_->publish(ros_message);
        RCLCPP_INFO_STREAM(get_logger(), "Send ros_message  value is = " + std::to_string(ros_message.data) + " to " +
                                         land_pub_->get_topic_name() + " success");
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in landCallback: " << std::string(e.what()));
    }
}

void MqttNode::returnCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in land.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        int mqtt_msg = std::make_unique<JsonUtils>()->msgIntValue(message);
        if (mqtt_msg != CtlState::Return) {
            RCLCPP_WARN_STREAM(get_logger(), "returnCallback: recv error input");
            return;
        }
        std_msgs::msg::Int32 ros_message;
        ros_message.data = mqtt_msg;
        return_pub_->publish(ros_message);
        RCLCPP_INFO_STREAM(get_logger(), "Send ros_message  value is = " + std::to_string(ros_message.data) + " to " +
                                         return_pub_->get_topic_name() + " success");
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in returnCallback: " << std::string(e.what()));
    }
}

void MqttNode::videoRecordCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in land.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        int mqtt_msg = std::make_unique<JsonUtils>()->msgIntValue(message);
        std_msgs::msg::Int8 ros_message;
        ros_message.data = mqtt_msg;
        videoRecord_pub_->publish(ros_message);
        RCLCPP_INFO_STREAM(get_logger(), "Send ros_message  value is = " + std::to_string(ros_message.data) + " to " +
                                         videoRecord_pub_->get_topic_name() + " success");
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in videoRecordCallback: " << std::string(e.what()));
    }
}


void MqttNode::shipRecordCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in land.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        int mqtt_msg = std::make_unique<JsonUtils>()->msgIntValue(message);
        std_msgs::msg::Bool ros_message;
        if (mqtt_msg == 1) {
            ros_message.data = true;
        } else if (mqtt_msg == -1) {
            ros_message.data = false;
        }
        shipRecord_pub_->publish(ros_message);
        RCLCPP_INFO_STREAM(get_logger(), "Send ros_message  value is = " + std::to_string(ros_message.data) + " to " +
                                         shipRecord_pub_->get_topic_name() + " success");
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in shipRecordCallback: " << std::string(e.what()));
    }
}

void MqttNode::lightControlCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in land.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        int mqtt_msg = std::make_unique<JsonUtils>()->msgIntValue(message);
        std_msgs::msg::Int8 ros_message;
        ros_message.data = mqtt_msg;
        lightControl_pub_->publish(ros_message);
        RCLCPP_INFO_STREAM(get_logger(), "Send ros_message  value is = " + std::to_string(ros_message.data) + " to " +
                                         lightControl_pub_->get_topic_name() + " success");
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in lightControlCallback: " << std::string(e.what()));
    }
}

void MqttNode::attackImgsCtlCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in attackImgsCtl.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        int mqtt_msg = std::make_unique<JsonUtils>()->msgIntValue(message);
        if (mqtt_msg == 1) {
            RCLCPP_INFO_STREAM(get_logger(), "start attack imgs send");
            is_send_attack_imgs_ = true;
        } else if (mqtt_msg == 0) {
            RCLCPP_INFO_STREAM(get_logger(), "stop attack imgs send");
            is_send_attack_imgs_ = false;
        } else {
            RCLCPP_ERROR_STREAM(get_logger(), "recv unknow attack imgs ctl: " << mqtt_msg);
        }
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in attackImgsCtlCallback: " << std::string(e.what()));
    }
}

void MqttNode::streamFlagCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in streamFlag.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        int mqtt_msg = std::make_unique<JsonUtils>()->msgIntValue(message);
        std_msgs::msg::Float32 ros_message;
        if (mqtt_msg == 1) {
            RCLCPP_INFO_STREAM(get_logger(), "start stream send");
            ros_message.data = stream_code_rate_;
            streamFlag_pub_->publish(ros_message);
        } else if (mqtt_msg == 0) {
            RCLCPP_INFO_STREAM(get_logger(), "stop stream send");
            ros_message.data = -1.0;
            streamFlag_pub_->publish(ros_message);
        } else {
            RCLCPP_ERROR_STREAM(get_logger(), "recv unknow stream flag: " << mqtt_msg);
        }
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in streamFlagCallback: " << std::string(e.what()));
    }
}

void MqttNode::ctlHeartbeatCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in ctlHeartbeat.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        // RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        recv_ctl_heartbeat_time_ = this->now();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in ctlHeartbeatCallback: " << std::string(e.what()));
    }
}

void MqttNode::relayCtlCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in relayCtl.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        float mqtt_msg = std::make_unique<JsonUtils>()->jsonRelayCtl(message);
        std_msgs::msg::Float32 ros_message;
        ros_message.data = mqtt_msg;
        relayHeight_pub_->publish(ros_message);
        RCLCPP_INFO_STREAM(get_logger(), "Send ros_message  value is = " + std::to_string(ros_message.data) + " to " +
                                         relayHeight_pub_->get_topic_name() + " success");
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in relayCtlCallback: " << std::string(e.what()));
    }
}

void MqttNode::taskDronesAdjustCallback(const std::string &topic, const std::string &message,
                                        const mqtt::properties &props) {
    // 检查消息是否为空
    std::string response_topic;
    std::string correlation_data;
    if (!checkMqttConditions(topic, message, props, response_topic, correlation_data)) {
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        msg_set_msgs::msg::AdjustTasks adjustTasks = std::make_unique<JsonUtils>()->jsonAdjustTask(message);
        adjustTasks_pub_->publish(adjustTasks);
        RCLCPP_INFO_STREAM(get_logger(),
                           "Send ros_message taskDronesAdjustCallback to " << adjustTasks_pub_->get_topic_name()
                                                                           << " success");
        mqttClientResponses(response_topic, correlation_data, MqttResponseCode::Success);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in taskDronesAdjustCallback: " << std::string(e.what()));
    }
}

void MqttNode::taskStreamCallback(const std::string &topic, const std::string &message, const mqtt::properties &props) {
    std::string response_topic;
    std::string correlation_data;
    if (!checkMqttConditions(topic, message, props, response_topic, correlation_data)) {
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        msg_set_msgs::msg::MultiTask multiTask = std::make_unique<JsonUtils>()->jsonTaskStream(message);
        multiTask_pub_->publish(multiTask);
        RCLCPP_INFO_STREAM(get_logger(),
                           "Send ros_message taskStream to " << multiTask_pub_->get_topic_name() << " success");
        mqttClientResponses(response_topic, correlation_data, MqttResponseCode::Success);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in taskStreamCallback: " << std::string(e.what()));
    }
}

void MqttNode::shipTaskCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in shipTaskCallback.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        msg_set_msgs::msg::MultiTask multiTask = std::make_unique<JsonUtils>()->jsonTaskStream(message);
        multiTask_pub_->publish(multiTask);
        RCLCPP_INFO_STREAM(get_logger(),
                           "Send ros_message taskStream to " << multiTask_pub_->get_topic_name() << " success");
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in shipTaskCallback: " << std::string(e.what()));
    }
}

void MqttNode::taskReturnCallback(const std::string &topic, const std::string &message, const mqtt::properties &props) {
    std::string response_topic;
    std::string correlation_data;
    if (!checkMqttConditions(topic, message, props, response_topic, correlation_data)) {
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        msg_set_msgs::msg::MultiTask multiTask = std::make_unique<JsonUtils>()->jsonTaskStream(message);
        taskReturn_pub_->publish(multiTask);
        RCLCPP_INFO_STREAM(get_logger(),
                           "Send ros_message taskReturn to " << taskReturn_pub_->get_topic_name() << " success");
        mqttClientResponses(response_topic, correlation_data, MqttResponseCode::Success);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in taskReturnCallback: " << std::string(e.what()));
    }
}

void
MqttNode::taskStreamJumpCallback(const std::string &topic, const std::string &message, const mqtt::properties &props) {
    std::string response_topic;
    std::string correlation_data;
    if (!checkMqttConditions(topic, message, props, response_topic, correlation_data)) {
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        msg_set_msgs::msg::JumpTask jumpTask = std::make_unique<JsonUtils>()->jsonTaskStreamJump(message);
        jumpTask_pub_->publish(jumpTask);
        RCLCPP_INFO_STREAM(get_logger(),
                           "Send ros_message taskStreamJump to " << jumpTask_pub_->get_topic_name() << " success");
        mqttClientResponses(response_topic, correlation_data, MqttResponseCode::Success);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in taskStreamJumpCallback: " << std::string(e.what()));
    }
}

void
MqttNode::taskStreamPauseCallback(const std::string &topic, const std::string &message, const mqtt::properties &props) {
    std::string response_topic;
    std::string correlation_data;
    if (!checkMqttConditions(topic, message, props, response_topic, correlation_data)) {
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        std_msgs::msg::Bool taskStreamPause = std::make_unique<JsonUtils>()->jsonTaskPause(message);
        taskPause_pub_->publish(taskStreamPause);
        RCLCPP_INFO_STREAM(get_logger(),
                           "Send ros_message taskStreamPause to " << taskPause_pub_->get_topic_name() << " success");
        mqttClientResponses(response_topic, correlation_data, MqttResponseCode::Success);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in taskStreamPauseCallback: " << std::string(e.what()));
    }
}

void MqttNode::followCoverAddTaskCallback(const std::string &topic, const std::string &message,
                                          const mqtt::properties &props) {
    std::string response_topic;
    std::string correlation_data;
    if (!checkMqttConditions(topic, message, props, response_topic, correlation_data)) {
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        msg_set_msgs::msg::CoverTask coverTask = std::make_unique<JsonUtils>()->jsonCoverTask(message);
        coverTask_pub_->publish(coverTask);
        RCLCPP_INFO_STREAM(get_logger(),
                           "Send ros_message followCoverAddTask to " << coverTask_pub_->get_topic_name() << " success");
        mqttClientResponses(response_topic, correlation_data, MqttResponseCode::Success);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in followCoverAddTaskCallback: " << std::string(e.what()));
    }
}

void MqttNode::followCoverParamAdjustCallback(const std::string &topic, const std::string &message,
                                              const mqtt::properties &props) {
    std::string response_topic;
    std::string correlation_data;
    if (!checkMqttConditions(topic, message, props, response_topic, correlation_data)) {
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        msg_set_msgs::msg::CoverParam coverParam = std::make_unique<JsonUtils>()->jsonCoverParam(message);
        coverParam_pub_->publish(coverParam);
        RCLCPP_INFO_STREAM(get_logger(),
                           "Send ros_message followCoverParamAdjust to " << coverParam_pub_->get_topic_name()
                                                                         << " success");
        mqttClientResponses(response_topic, correlation_data, MqttResponseCode::Success);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in followCoverParamAdjustCallback: " << std::string(e.what()));
    }
}

void MqttNode::followCoverPathAdjustCallback(const std::string &topic, const std::string &message,
                                             const mqtt::properties &props) {
    std::string response_topic;
    std::string correlation_data;
    if (!checkMqttConditions(topic, message, props, response_topic, correlation_data)) {
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        mavros_msgs::msg::WaypointList coverPath = std::make_unique<JsonUtils>()->jsonCoverPath(message);
        coverPath_pub_->publish(coverPath);
        RCLCPP_INFO_STREAM(get_logger(),
                           "Send ros_message followCoverPathAdjust to " << coverPath_pub_->get_topic_name()
                                                                        << " success");
        mqttClientResponses(response_topic, correlation_data, MqttResponseCode::Success);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in followCoverPathAdjustCallback: " << std::string(e.what()));
    }
}

void MqttNode::followCoverTaskPauseCallback(const std::string &topic, const std::string &message,
                                            const mqtt::properties &props) {
    std::string response_topic;
    std::string correlation_data;
    if (!checkMqttConditions(topic, message, props, response_topic, correlation_data)) {
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        std_msgs::msg::Bool coverTaskPause = std::make_unique<JsonUtils>()->jsonTaskPause(message);
        taskPause_pub_->publish(coverTaskPause);
        RCLCPP_INFO_STREAM(get_logger(), "Send ros_message followCoverTaskPause to " << taskPause_pub_->get_topic_name()
                                                                                     << " success");
        mqttClientResponses(response_topic, correlation_data, MqttResponseCode::Success);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in followCoverTaskPauseCallback: " << std::string(e.what()));
    }
}

void MqttNode::followCoverWaypointJumpCallback(const std::string &topic, const std::string &message,
                                               const mqtt::properties &props) {
    std::string response_topic;
    std::string correlation_data;
    if (!checkMqttConditions(topic, message, props, response_topic, correlation_data)) {
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        std_msgs::msg::Float32MultiArray coverGoalPoint = std::make_unique<JsonUtils>()->jsonCoverGoalPoint(message);
        coverJump_pub_->publish(coverGoalPoint);
        RCLCPP_INFO_STREAM(get_logger(),
                           "Send ros_message followCoverWaypointJump to " << coverJump_pub_->get_topic_name()
                                                                          << " success");
        mqttClientResponses(response_topic, correlation_data, MqttResponseCode::Success);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in followCoverWaypointJumpCallback: " << std::string(e.what()));
    }
}

void MqttNode::followCoverGlobalParamCallback(const std::string &topic, const std::string &message,
                                              const mqtt::properties &props) {
    std::string response_topic;
    std::string correlation_data;
    if (!checkMqttConditions(topic, message, props, response_topic, correlation_data)) {
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        msg_set_msgs::msg::GlobalParam globalParam = std::make_unique<JsonUtils>()->jsonGlobalParam(message);
        globalParam_pub_->publish(globalParam);
        RCLCPP_INFO_STREAM(get_logger(),
                           "Send ros_message followCoverGlobalParam to " << globalParam_pub_->get_topic_name()
                                                                         << " success");
        mqttClientResponses(response_topic, correlation_data, MqttResponseCode::Success);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in followCoverGlobalParamCallback: " << std::string(e.what()));
    }
}

void MqttNode::followCoverTaskCancelCallback(const std::string &topic, const std::string &message,
                                             const mqtt::properties &props) {
    std::string response_topic;
    std::string correlation_data;
    if (!checkMqttConditions(topic, message, props, response_topic, correlation_data)) {
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        std_msgs::msg::Bool cancelTask = std::make_unique<JsonUtils>()->jsonCancelTask(message);
        cancelTask_pub_->publish(cancelTask);
        RCLCPP_INFO_STREAM(get_logger(),
                           "Send ros_message followCoverTaskCancel to " << cancelTask_pub_->get_topic_name()
                                                                        << " success");
        mqttClientResponses(response_topic, correlation_data, MqttResponseCode::Success);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in followCoverTaskCancelCallback: " << std::string(e.what()));
    }
}

void MqttNode::followCoverDroneGroupAdjustCallback(const std::string &topic, const std::string &message,
                                                   const mqtt::properties &props) {
    std::string response_topic;
    std::string correlation_data;
    if (!checkMqttConditions(topic, message, props, response_topic, correlation_data)) {
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        std_msgs::msg::Int8MultiArray dronesId = std::make_unique<JsonUtils>()->jsonDronesAdjust(message);
        // dronesAdjust_pub_->publish(dronesId);
        // RCLCPP_INFO_STREAM(get_logger(),
        //                    "Send ros_message followCoverDroneGroupAdjust to " << dronesAdjust_pub_->get_topic_name()
        //                                                                       << " success");
        mqttClientResponses(response_topic, correlation_data, MqttResponseCode::Success);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in followCoverDroneGroupAdjustCallback: " << std::string(e.what()));
    }
}

void MqttNode::multiGoalCallback(const std::string &topic, const std::string &message, const mqtt::properties &props) {
    std::string response_topic;
    std::string correlation_data;
    if (!checkMqttConditions(topic, message, props, response_topic, correlation_data)) {
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        msg_set_msgs::msg::MultiGoal multiGoal = std::make_unique<JsonUtils>()->jsonMultiGoal(message);
        multiGoal_pub_->publish(multiGoal);
        RCLCPP_INFO_STREAM(get_logger(),
                           "Send ros_message multiGoalCallback to " << multiGoal_pub_->get_topic_name() << " success");
        mqttClientResponses(response_topic, correlation_data, MqttResponseCode::Success);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in multiGoalCallback: " << std::string(e.what()));
    }
}

void MqttNode::refGpsCallback(const std::string &topic, const std::string &message, const mqtt::properties &props) {
    std::string response_topic;
    std::string correlation_data;
    // std::cout<<"callback refgps"<<std::endl;
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        sensor_msgs::msg::NavSatFix refGps = std::make_unique<JsonUtils>()->jsonrefGps(message);
        refGps_pub_->publish(refGps);
        RCLCPP_INFO_STREAM(get_logger(), "Send ros_message refGps to " << refGps_pub_->get_topic_name() << " success");
        // mqttClientResponses(response_topic, correlation_data, MqttResponseCode::Success);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in refGpsCallback: " << std::string(e.what()));
    }
}

void MqttNode::ptzControlCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in ptzControlCallback.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        std_msgs::msg::Int32MultiArray pub_msg = std::make_unique<JsonUtils>()->jsonToInt32Array(message);
        ptzControl_pub_->publish(pub_msg);
        RCLCPP_INFO_STREAM(get_logger(), "Send ros_message to " << ptzControl_pub_->get_topic_name() << " success");
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in ptzControlCallback: " << std::string(e.what()));
    }
}

void MqttNode::imageGetCallback(const std::string &topic, const std::string &message) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in imageGetCallback.");
        return;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message.");
        return;
    }
    try {
        RCLCPP_INFO_STREAM(get_logger(), "Received mqtt message from " << topic << " :" << message);
        uint8_t mqtt_msg = std::make_unique<JsonUtils>()->msgUInt8Value(message);
        std_msgs::msg::UInt8 pub_msg;
        pub_msg.data = mqtt_msg;
        imageGet_pub_->publish(pub_msg);
        RCLCPP_INFO_STREAM(get_logger(), "Send ros_message  value is = " + std::to_string(pub_msg.data) + " to " +
                                         imageGet_pub_->get_topic_name() + " success");
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in imageGetCallback: " << std::string(e.what()));
    }
}

// ------------------------ MQTT callback: recv MQTT, and send with ROS --------------------------------

void MqttNode::subscribeTopics() {
    // MQTT subscriber
    // uav ctl
    subscribe(MqttTopic::mqtt_uav_ctl_state + std::to_string(vehicle_id_),
              std::bind(&MqttNode::ctrStateCallback, this, std::placeholders::_1, std::placeholders::_2));
    subscribe(MqttTopic::mqtt_uav_armed + std::to_string(vehicle_id_),
              std::bind(&MqttNode::armedCallback, this, std::placeholders::_1, std::placeholders::_2));
    subscribe(MqttTopic::mqtt_uav_takeoff + std::to_string(vehicle_id_),
              std::bind(&MqttNode::takeOffCallback, this, std::placeholders::_1, std::placeholders::_2));
    subscribe(MqttTopic::mqtt_uav_hover + std::to_string(vehicle_id_),
              std::bind(&MqttNode::hoverCallback, this, std::placeholders::_1, std::placeholders::_2));
    subscribe(MqttTopic::mqtt_uav_land + std::to_string(vehicle_id_),
              std::bind(&MqttNode::landCallback, this, std::placeholders::_1, std::placeholders::_2));
    subscribe(MqttTopic::mqtt_uav_return + std::to_string(vehicle_id_),
              std::bind(&MqttNode::returnCallback, this, std::placeholders::_1, std::placeholders::_2));

    // attack imgs ctl
    subscribe(MqttTopic::mqtt_attack_imgs_ctl + std::to_string(vehicle_id_),
              std::bind(&MqttNode::attackImgsCtlCallback, this, std::placeholders::_1, std::placeholders::_2));

    // stream flag ctl
    subscribe(MqttTopic::mqtt_stream_flag + std::to_string(vehicle_id_),
              std::bind(&MqttNode::streamFlagCallback, this, std::placeholders::_1, std::placeholders::_2));

    // ctl heartbeat
    subscribe(MqttTopic::mqtt_ctl_heartbeat,
              std::bind(&MqttNode::ctlHeartbeatCallback, this, std::placeholders::_1, std::placeholders::_2));

    // relay ctl
    subscribe(MqttTopic::mqtt_delay_ctl + std::to_string(vehicle_id_),
              std::bind(&MqttNode::relayCtlCallback, this, std::placeholders::_1, std::placeholders::_2));

    // video record
    subscribe(MqttTopic::mqtt_video_record + std::to_string(vehicle_id_),
              std::bind(&MqttNode::videoRecordCallback, this, std::placeholders::_1, std::placeholders::_2));

    subscribe(MqttTopic::mqtt_light_control + std::to_string(vehicle_id_),
              std::bind(&MqttNode::lightControlCallback, this, std::placeholders::_1, std::placeholders::_2));

    subscribe(MqttTopic::mqtt_ship_task + std::to_string(vehicle_id_),
              std::bind(&MqttNode::shipTaskCallback, this, std::placeholders::_1, std::placeholders::_2));

    subscribe(MqttTopic::mqtt_ship_record + std::to_string(vehicle_id_),
              std::bind(&MqttNode::shipRecordCallback, this, std::placeholders::_1, std::placeholders::_2));

    // task ctl
    subscribeWithProps(MqttTopic::mqtt_request_task_deploy_task_stream + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::taskStreamCallback, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3));
    subscribeWithProps(MqttTopic::mqtt_request_task_deploy_return + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::taskReturnCallback, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3));
    subscribeWithProps(MqttTopic::mqtt_request_task_deploy_stream_jump + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::taskStreamJumpCallback, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3));
    subscribeWithProps(MqttTopic::mqtt_request_task_deploy_stream_pause + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::taskStreamPauseCallback, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3));
    subscribeWithProps(MqttTopic::mqtt_request_follow_cover_add_task + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::followCoverAddTaskCallback, this, std::placeholders::_1,
                                 std::placeholders::_2, std::placeholders::_3));
    subscribeWithProps(MqttTopic::mqtt_request_follow_cover_param_adjust + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::followCoverParamAdjustCallback, this, std::placeholders::_1,
                                 std::placeholders::_2, std::placeholders::_3));
    subscribeWithProps(MqttTopic::mqtt_request_follow_cover_path_adjust + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::followCoverPathAdjustCallback, this, std::placeholders::_1,
                                 std::placeholders::_2, std::placeholders::_3));
    subscribeWithProps(MqttTopic::mqtt_request_follow_cover_task_pause + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::followCoverTaskPauseCallback, this, std::placeholders::_1,
                                 std::placeholders::_2, std::placeholders::_3));
    subscribeWithProps(MqttTopic::mqtt_request_follow_cover_waypoint_jump + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::followCoverWaypointJumpCallback, this, std::placeholders::_1,
                                 std::placeholders::_2, std::placeholders::_3));
    subscribeWithProps(MqttTopic::mqtt_request_follow_cover_global_param + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::followCoverGlobalParamCallback, this, std::placeholders::_1,
                                 std::placeholders::_2, std::placeholders::_3));
    subscribeWithProps(MqttTopic::mqtt_request_follow_cover_task_cancel + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::followCoverTaskCancelCallback, this, std::placeholders::_1,
                                 std::placeholders::_2, std::placeholders::_3));
    subscribeWithProps(MqttTopic::mqtt_request_follow_cover_drone_group_adjust + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::followCoverDroneGroupAdjustCallback, this, std::placeholders::_1,
                                 std::placeholders::_2, std::placeholders::_3));

    subscribeWithProps(MqttTopic::mqtt_request_ref_gps,
                       std::bind(&MqttNode::refGpsCallback, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3));

    subscribeWithProps(MqttTopic::mqtt_request_multi_goal + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::multiGoalCallback, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3));

    subscribeWithProps(MqttTopic::mqtt_request_task_drones_adjust + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::taskDronesAdjustCallback, this, std::placeholders::_1,
                                 std::placeholders::_2,
                                 std::placeholders::_3));

    // ptz
    subscribeWithProps(MqttTopic::mqtt_ptz_control + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::ptzControlCallback, this, std::placeholders::_1, std::placeholders::_2));
    // save_image
    subscribeWithProps(MqttTopic::mqtt_image_get + std::to_string(vehicle_id_),
                       std::bind(&MqttNode::imageGetCallback, this, std::placeholders::_1, std::placeholders::_2));

}

bool MqttNode::checkMqttConditions(const std::string &request_topic, const std::string &message,
                                   const mqtt::properties &props, std::string &response_topic,
                                   std::string &correlation_data) {
    if (message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in " << request_topic);
        return false;
    }
    // 检查客户端连接状态
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot subscribe to the message");
        return false;
    }

    // 检查 property 是否包含 RESPONSE_TOPIC 和 CORRELATION_DATA
    if (!props.contains(mqtt::property::RESPONSE_TOPIC) || !props.contains(mqtt::property::CORRELATION_DATA)) {
        RCLCPP_WARN_STREAM(get_logger(), "Required properties (RESPONSE_TOPIC or CORRELATION_DATA) are missing");
        return false;
    }

    // 提取 RESPONSE_TOPIC
    response_topic = mqtt::get<std::string>(props, mqtt::property::RESPONSE_TOPIC);

    // 提取 CORRELATION_DATA
    correlation_data = mqtt::get<std::string>(props, mqtt::property::CORRELATION_DATA);

    return true;
}

void MqttNode::mqttClientResponses(const std::string &response_topic, const std::string &correlation_data,
                                   MqttResponseCode code) {
    if (!is_connected_) {
        RCLCPP_WARN_STREAM(get_logger(), "MQTT client is not connected. Cannot publish the response message");
        return;
    }
    try {
        std::string jsonData = JsonUtils::jsonValue(code);
        auto responseMsg = mqtt::make_message(response_topic, jsonData, 2, false);
        mqtt::properties response_props;
        response_props.add(mqtt::property(mqtt::property::CORRELATION_DATA, correlation_data));
        responseMsg->set_properties(response_props);
        client_->publish(responseMsg);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in mqttClientResponses: " << std::string(e.what()));
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Mqtt exception: " << e.what());
    }
}

MqttNode::~MqttNode() {
    client_->disconnect();
    rclcpp::shutdown();
}

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//
//     auto time = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
//     time_t timestamp = time.time_since_epoch().count();
//
// #ifdef NDEBUG
//     std::cout << "release --------------------------------------- " << std::endl;
//     // mqttnode yaml
//     std::string config_file_path = "/etc/mqttnode.yaml";
//     YAML::Node config;
//     try {
//         config = YAML::LoadFile(config_file_path);
//     } catch (const YAML::BadFile &e) {
//         std::cerr << "Failed to load config file: " << config_file_path << std::endl;
//         return -1;
//     }
//     std::string address = config["address"].as<std::string>("tcp://127.0.0.1:1883");
//     std::string mqtt_username = config["mqtt_username"].as<std::string>("admin");
//     std::string mqtt_passwd = config["mqtt_passwd"].as<std::string>("nanhu@2020");
//     int mqtt_timeout = config["mqtt_timeout"].as<int>(60);
//     int mqtt_keep_alive_interval = config["mqtt_keep_alive_interval"].as<int>(100);
//     int mqtt_buffer_size = config["mqtt_buffer_size"].as<int>(31457280);
//     std::string prefix = config["prefix"].as<std::string>("drone_");
//     int vehicle_id = config["vehicle_id"].as<int>(0);
//     double pub_rate = config["pub_rate"].as<double>(5.0);
//
//     std::string client_id = std::to_string(timestamp) + "pubsub" + std::to_string(vehicle_id);
//     std::string curr_drone_id = "/" + prefix + std::to_string(vehicle_id);
//
//     std::string node_name = "mqttnode_foxy_node_" + std::to_string(vehicle_id);
//     std::string node_namespace = prefix + std::to_string(vehicle_id);
//
//     auto mqtt_node = std::make_shared<MqttNode>(address, client_id, mqtt_username, mqtt_passwd, mqtt_timeout,
//         mqtt_keep_alive_interval, mqtt_buffer_size, prefix, curr_drone_id, vehicle_id, pub_rate, node_name, node_namespace);
//
//     mqtt_node->connect();
//
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(mqtt_node);
//     executor.spin();
//
//     rclcpp::shutdown();
// #else
//     std::cout << "debug --------------------------------------- " << std::endl;
//     auto mqtt_node = std::make_shared<MqttNode>();
//
//     mqtt_node->connect();
//
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(mqtt_node);
//     executor.spin();
//
//     rclcpp::shutdown();
// #endif
//     return 0;
// }
