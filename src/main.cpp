#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "catching_blimp.h"

// Error handle loop
void error_loop() {
    while (1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Global variables and instances (To be initialized properly)
std::string blimpNameSpace;
std_msgs::msg::String identity_msg;
std_msgs::msg::String log_msg;
std_msgs::msg::Float64 height_msg;
std_msgs::msg::Float64 z_velocity_msg;
std_msgs::msg::Int64 state_machine_msg;
std_msgs::msg::Float64MultiArray debug_msg;

class BlimpNode : public rclcpp::Node {
public:
    BlimpNode() : Node("blimp_node") {
        // Create publishers
        identity_publisher_ = this->create_publisher<std_msgs::msg::String>("/identify", 10);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(blimpNameSpace + "/imu", 10);
        debug_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(blimpNameSpace + "/debug", 10);
        height_publisher_ = this->create_publisher<std_msgs::msg::Float64>(blimpNameSpace + "/height", 10);
        z_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>(blimpNameSpace + "/z_velocity", 10);
        state_machine_publisher_ = this->create_publisher<std_msgs::msg::Int64>(blimpNameSpace + "/state_machine", 10);
        log_publisher_ = this->create_publisher<std_msgs::msg::String>(blimpNameSpace + "/log", 10);

        // Create subscriptions
        auto_subscription_ = this->create_subscription<std_msgs::msg::Bool>(blimpNameSpace + "/auto", 10,
            std::bind(&BlimpNode::auto_subscription_callback, this, std::placeholders::_1));
        baseBarometer_subscription_ = this->create_subscription<std_msgs::msg::Float64>(blimpNameSpace + "/baseBarometer", 10,
            std::bind(&BlimpNode::baseBarometer_subscription_callback, this, std::placeholders::_1));
        calibrateBarometer_subscription_ = this->create_subscription<std_msgs::msg::Bool>(blimpNameSpace + "/calibrateBarometer", 10,
            std::bind(&BlimpNode::calibrateBarometer_subscription_callback, this, std::placeholders::_1));
        grabber_subscription_ = this->create_subscription<std_msgs::msg::Bool>(blimpNameSpace + "/grabbing", 10,
            std::bind(&BlimpNode::grab_subscription_callback, this, std::placeholders::_1));
        shooter_subscription_ = this->create_subscription<std_msgs::msg::Bool>(blimpNameSpace + "/shooting", 10,
            std::bind(&BlimpNode::shoot_subscription_callback, this, std::placeholders::_1));
        motor_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(blimpNameSpace + "/motorCommands", 10,
            std::bind(&BlimpNode::motor_subscription_callback, this, std::placeholders::_1));
        kill_subscription_ = this->create_subscription<std_msgs::msg::Bool>(blimpNameSpace + "/killed", 10,
            std::bind(&BlimpNode::kill_subscription_callback, this, std::placeholders::_1));
        goal_color_subscription_ = this->create_subscription<std_msgs::msg::Int64>(blimpNameSpace + "/goal_color", 10,
            std::bind(&BlimpNode::goal_color_subscription_callback, this, std::placeholders::_1));
        targets_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(blimpNameSpace + "/targets", 10,
            std::bind(&BlimpNode::targets_subscription_callback, this, std::placeholders::_1));
        pixels_subscription_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(blimpNameSpace + "/pixels", 10,
            std::bind(&BlimpNode::pixels_subscription_callback, this, std::placeholders::_1));
        avoidance_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(blimpNameSpace + "/avoidance", 10,
            std::bind(&BlimpNode::avoidance_subscription_callback, this, std::placeholders::_1));

        // Create timers
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&BlimpNode::timer_callback, this));

        // Initialize other member variables
        init_messages();
    }

private:
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr identity_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr height_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr z_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr state_machine_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_publisher_;

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr baseBarometer_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr calibrateBarometer_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grabber_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shooter_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motor_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr goal_color_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr targets_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr pixels_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr avoidance_subscription_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    void init_messages() {
        // Initialize messages
        identity_msg.data = blimpNameSpace;
        log_msg.data = "";
        height_msg.data = 0.0;
        z_velocity_msg.data = 0.0;
        state_machine_msg.data = 0;
        debug_msg.data.clear();
    }

    void publish_log(const std::string &message) {
        log_msg.data = message;
        log_publisher_->publish(log_msg);
    }

    void timer_callback() {
        identity_publisher_->publish(identity_msg);
    }

    void auto_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Bool *auto_msg = (const std_msgs__msg__Bool *)msgin;
    if (auto_msg->data) {
        if (blimp_state == manual) {
            publish_log("Activating Auto Mode");
        }
        blimp_state = autonomous;
    } else {
        if (blimp_state == autonomous) {
            publish_log("Going Manual for a Bit...");
        }
        blimp_state = manual;
    }
}

void calibrateBarometer_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Bool *calibration_msg = (const std_msgs__msg__Bool *)msgin;
    calibrateBaro = calibration_msg->data;
    const char * boolAsConstCharPtr = calibrateBaro ? "true" : "false";
    // publish_log(boolAsConstCharPtr);

    // Barometer Calibration
    if (calibrateBaro == true) {
        baroCalibrationOffset = BerryIMU.comp_press - baseBaro;

        std::string floatAsString = std::to_string(BerryIMU.comp_press);
        const char* floatAsConstCharPtr = floatAsString.c_str();
        publish_log(floatAsConstCharPtr);
        std::string floatAsString2 = std::to_string(baseBaro);
        const char* floatAsConstCharPtr2 = floatAsString2.c_str();
        publish_log(floatAsConstCharPtr2);
        std::string floatAsString3 = std::to_string(baroCalibrationOffset);
        const char* floatAsConstCharPtr3 = floatAsString3.c_str();
        publish_log(floatAsConstCharPtr3);
        publish_log("Calibrating Barometer");
    }
}

void baro_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Float64 *baro_msg = (const std_msgs__msg__Float64 *)msgin;
    baseBaro = baro_msg->data;
    
    //heartbeat
    //update last message time
    lastMsgTime = micros()/MICROS_TO_SEC;

    //If teensy comes out of lost blimp_state, put it in manual control mode
    if (blimp_state == lost) {
        blimp_state = manual;
    }
}

void grab_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Bool *grab_msg = (const std_msgs__msg__Bool *)msgin;
    if (grabCom == 0 && grab_msg->data) {
        grabCom = 1;
        publish_log("Going for a catch...");
    } else if (grabCom == 1 && !grab_msg->data) {
        grabCom = 0;
        publish_log("Hopefully I got a balloon!");
    }
}

void kill_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Bool *kill_msg = (const std_msgs__msg__Bool *)msgin;

    if (kill_msg->data == true) {
        publish_log("I'm ded xD");
        motorControl.update(0,0,0,0,0);
        bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
        bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
        leftGimbal.updateGimbal(leftReady && rightReady);
        rightGimbal.updateGimbal(leftReady && rightReady);
    }
}

void shoot_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Bool *shoot_msg = (const std_msgs__msg__Bool *)msgin;
    if (shootCom == 0 && shoot_msg->data) {
        shootCom = 1;
        publish_log("I'm shooting my shot...");
    } else if (shootCom == 1 && !shoot_msg->data) {
        shootCom = 0;
        publish_log("What a shot!");
    }
}

void motor_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Float64MultiArray *motor_msg = (const std_msgs__msg__Float64MultiArray *)msgin;

    //commands from basestation
    forward_msg = motor_msg->data.data[3];
    up_msg = motor_msg->data.data[1];
    yaw_msg = motor_msg->data.data[0];
    translation_msg = motor_msg->data.data[2];

    // char motorCommands[100];  // Size depending on the expected maximum length of your combined string
    // sprintf(motorCommands, "Teensy Motor Commands\nYaw: %.2f\nUp: %.2f\nTranslation: %.2f\nForward: %.2f\n", yaw_msg, up_msg, translation_msg, forward_msg);
    // publish_log(motorCommands);
}

void goal_color_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Int64 *goal_color_msg = (const std_msgs__msg__Int64 *)msgin;
    int goal_color = goal_color_msg->data;
    if (goalColor != orange && goal_color == 0) {
        goalColor = orange;
        publish_log("Goal Color changed to Orange");
    } else if (goalColor != yellow && goal_color == 1) {
        goalColor = yellow;
        publish_log("Goal Color changed to Yellow");
    }
}

void avoidance_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Float64MultiArray *avoidance_msg = (const std_msgs__msg__Float64MultiArray *)msgin;
    //3 objects with xyz (9 elements in total)
    for (size_t i = 0; i < 9; ++i) {
        avoidance[i] = avoidance_msg->data.data[i];
    }
}

void targets_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Float64MultiArray *targets_msg = (const std_msgs__msg__Float64MultiArray *)msgin;
    // object of interest with xyz (3 elements in total)
    for (size_t i = 0; i < 3; ++i) {
        targets[i] = targets_msg->data.data[i];
    }
}

void pixels_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Int64MultiArray *pixels_msg = (const std_msgs__msg__Int64MultiArray *)msgin;
    //3 objects with xyz (9 elements in total)
    for (size_t i = 0; i < 9; ++i) {
        pixels[i] = pixels_msg->data.data[i];
    }
}
void Pulse() {
    if (digitalRead(interruptPin) == HIGH) {
        // start measuring
        pulseInTimeBegin = micros();
    }
    else {
        // stop measuring
        pulseInTimeEnd = micros();
        newPulseDurationAvailable = true;
    }
}

float searchDirection() {
    int rand = random(0, 10);
    float binary = 1.0;
    if (rand <= 4){
        binary = 1.0;
    }else if (rand > 4){
        binary = -1.0;
    }
    return binary;
}

void setup() { // TODO: Convert to C++
    //start serial connection
    Serial.begin(115200);
    set_microros_serial_transports(Serial); //to pi

    agent_state_ = WAITING_AGENT; //wait for connection
    Serial1.begin(115200);

    //Start IMU
    BerryIMU.BerryIMU_v3_Setup();

    //initialize messages
    //service message
    // req.string_value.data = (char *)malloc(BUFFER_LEN * sizeof(char));

    //publisher/subscriber messages
    //See https://micro.ros.org/docs/tutorials/advanced/handling_type_memory
    //String type
    identity_msg.data.capacity = BUFFER_LEN;
    identity_msg.data.data = (char *) malloc(identity_msg.data.capacity*sizeof(char));
    identity_msg.data.size = 0;

    //Once identity_msg is initialized, we can set it to its constant value (blimp ID / namespace)
    snprintf(identity_msg.data.data, BUFFER_LEN, "%s", blimpNameSpace.c_str());
    identity_msg.data.size = strlen(identity_msg.data.data);
    identity_msg.data.capacity = BUFFER_LEN;

    log_msg.data.capacity = BUFFER_LEN;
    log_msg.data.data = (char *) malloc(log_msg.data.capacity*sizeof(char));
    log_msg.data.size = 0;

    //IMU Header
    imu_msg.header.frame_id.capacity = BUFFER_LEN;
    imu_msg.header.frame_id.data = (char *) malloc(BUFFER_LEN*sizeof(char));
    imu_msg.header.frame_id.size = 0;

    //Float64MultiArray type
    motor_msg.data.capacity = 4;
    motor_msg.data.data = (double *) malloc(motor_msg.data.capacity*sizeof(double));
    motor_msg.data.size = 0;

    //Initialize MultiArray layout and inner members
    motor_msg.layout.dim.capacity = 100;
    motor_msg.layout.dim.size = 0;
    motor_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(motor_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
    for(size_t i = 0; i < motor_msg.layout.dim.capacity; i++){
        motor_msg.layout.dim.data[i].label.capacity = 20;
        motor_msg.layout.dim.data[i].label.size = 0;
        motor_msg.layout.dim.data[i].label.data = (char*) malloc(motor_msg.layout.dim.data[i].label.capacity * sizeof(char));
    }

    debug_msg.data.capacity = BUFFER_LEN;
    debug_msg.data.data = (double *) malloc(debug_msg.data.capacity*sizeof(double));
    debug_msg.data.size = sizeof(debug_msg.data.data);
    
    debug_msg.layout.dim.capacity = 100;
    debug_msg.layout.dim.size = 0;
    debug_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(debug_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
    for(size_t i = 0; i < debug_msg.layout.dim.capacity; i++){
        debug_msg.layout.dim.data[i].label.capacity = 20;
        debug_msg.layout.dim.data[i].label.size = 0;
        debug_msg.layout.dim.data[i].label.data = (char*) malloc(debug_msg.layout.dim.data[i].label.capacity * sizeof(char));
    }

    targets_msg.data.capacity = MAX_TARGETS;
    targets_msg.data.data = (double *) malloc(targets_msg.data.capacity*sizeof(double));
    targets_msg.data.size = sizeof(targets_msg.data.data);

    targets_msg.layout.dim.capacity = 100;
    targets_msg.layout.dim.size = 0;
    targets_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(targets_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
    for(size_t i = 0; i < targets_msg.layout.dim.capacity; i++){
        targets_msg.layout.dim.data[i].label.capacity = 20;
        targets_msg.layout.dim.data[i].label.size = 0;
        targets_msg.layout.dim.data[i].label.data = (char*) malloc(targets_msg.layout.dim.data[i].label.capacity * sizeof(char));
    }

    //Todo: Set capacity on these guys to something reasonable
    avoidance_msg.data.capacity = 100;
    avoidance_msg.data.data = (double *) malloc(avoidance_msg.data.capacity*sizeof(double));
    avoidance_msg.data.size = sizeof(avoidance_msg.data.data);

    avoidance_msg.layout.dim.capacity = 100;
    avoidance_msg.layout.dim.size = 0;
    avoidance_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(avoidance_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
    for(size_t i = 0; i < avoidance_msg.layout.dim.capacity; i++){
        avoidance_msg.layout.dim.data[i].label.capacity = 20;
        avoidance_msg.layout.dim.data[i].label.size = 0;
        avoidance_msg.layout.dim.data[i].label.data = (char*) malloc(avoidance_msg.layout.dim.data[i].label.capacity * sizeof(char));
    }

    //Int64MultiArray
    pixels_msg.data.capacity = 100;
    pixels_msg.data.data = (int64_t *) malloc(pixels_msg.data.capacity*sizeof(int64_t));
    pixels_msg.data.size = sizeof(pixels_msg.data.data);
    
    pixels_msg.layout.dim.capacity = 100;
    pixels_msg.layout.dim.size = 0;
    pixels_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(pixels_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
    for(size_t i = 0; i < pixels_msg.layout.dim.capacity; i++){
        pixels_msg.layout.dim.data[i].label.capacity = 20;
        pixels_msg.layout.dim.data[i].label.size = 0;
        pixels_msg.layout.dim.data[i].label.data = (char*) malloc(pixels_msg.layout.dim.data[i].label.capacity * sizeof(char));
    }

    //WDW I took this out, seems that there is no need for it
    // delay(2000);

    //time out
    firstMessageTime = micros()/MICROS_TO_SEC;
    
    //initialize
    ballGrabber.ballgrabber_init(GATE_S, PWM_G);
    leftGimbal.gimbal_init(L_Yaw, L_Pitch, PWM_L, 25, 30, MIN_MOTOR, MAX_MOTOR, 45, 0.5);
    rightGimbal.gimbal_init(R_Yaw, R_Pitch, PWM_R, 25, 30, MIN_MOTOR, MAX_MOTOR, 135, 0.5);
}

void update_agent_state() {
    // Update the microros agent state machine
    switch (agent_state_) {
        case WAITING_AGENT:
            if (rclcpp::ok() && rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                agent_state_ = AGENT_AVAILABLE;
            }
            break;
        case AGENT_AVAILABLE:
            if (create_entities()) {
                agent_state_ = AGENT_CONNECTED;
            } else {
                destroy_entities();
                agent_state_ = WAITING_AGENT;
            }
            break;
        case AGENT_CONNECTED:
            if (rclcpp::ok() && rmw_uros_ping_agent(10, 1) == RMW_RET_OK) {
                rclcpp::spin_some(this->get_node_base_interface());
            } else {
                destroy_entities();
                agent_state_ = AGENT_DISCONNECTED;
            }
            break;
        case AGENT_DISCONNECTED:
            destroy_entities();
            agent_state_ = WAITING_AGENT;
            break;
        default:
            break;
    }
}

//TODO: LOOP?