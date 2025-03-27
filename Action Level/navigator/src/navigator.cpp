#include "navigator/navigator.hpp"

Navigator::Navigator(const rclcpp::NodeOptions& options) : Node("navigator", options) {

    this->declare_parameter<std::string>("navigator_label", "red_standard_robot1");
    label_ = this->get_parameter("navigator_label").as_string();

    std::string attachment = label_;
    if (!attachment.empty()) {
        attachment += "/";
    }

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto sub_opt = rclcpp::SubscriptionOptions();
    auto pub_opt = rclcpp::PublisherOptions();
    sub_opt.callback_group = callback_group_;
    pub_opt.callback_group = callback_group_;

    nav_msg_sub_ = this->create_subscription<navigator_interfaces::msg::Navigate>(
        "to_navigator", 10, std::bind(&Navigator::nav_callback, this, std::placeholders::_1), sub_opt);
    current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(attachment + "navigator/current_pose", 10);

    nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, attachment + "navigate_to_pose", callback_group_);
    send_goal_options_.goal_response_callback = std::bind(&Navigator::goal_response_callback, this,
                                                          std::placeholders::_1);
    send_goal_options_.feedback_callback = std::bind(&Navigator::feedback_callback, this, std::placeholders::_1,
                                                     std::placeholders::_2);
    send_goal_options_.result_callback = std::bind(&Navigator::result_callback, this, std::placeholders::_1);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Navigator::timer_callback, this), callback_group_);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        attachment + "tf", 10, std::bind(&Navigator::tf2_listener_callback, this, std::placeholders::_1), sub_opt);

    endtime_ = std::chrono::steady_clock::now();
    nav_state_ = INIT;
    failed_count_ = 0;
    available_ = true;
}

void Navigator::goal_response_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future) {
    auto goal_handle = future;
    if (!goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Goal was rejected by server");
        available_ = true;
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
    goal_handle_ = goal_handle;
}

void Navigator::feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Current point: (%.2f, %.2f)", feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y);
    RCLCPP_INFO(this->get_logger(), "Distance to target point: %.2f m", feedback->distance_remaining);
}

void Navigator::result_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal was reached");
        nav_state_ = NavState::REACHED;
        available_ = true;
        failed_count_ = 0;
        endtime_ = std::chrono::steady_clock::now();
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(this->get_logger(), "Goal was aborted");
        nav_state_ = NavState::ABORTED;
        available_ = true;
        failed_count_++;
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        nav_state_ = NavState::CANCELED;
        available_ = true;
        failed_count_++;
        break;
    default:
        RCLCPP_INFO(get_logger(), "Unknown result code");
        nav_state_ = NavState::UNKNOWN;
        available_ = true;
        failed_count_++;
        break;
    }
}

void Navigator::tf2_listener_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    for (const auto& transform : msg->transforms) {
        tf2_buffer_->setTransform(transform, label_, true);
    }
}

void Navigator::nav_callback(const navigator_interfaces::msg::Navigate::SharedPtr msg) {
    if (msg->instant) {
        nav_cancel();
    }
    nav_to_pose(msg->pose);
}

void Navigator::timer_callback() {
    get_current_pose();
}

void Navigator::nav_to_pose(const geometry_msgs::msg::PoseStamped& msg) {
    RCLCPP_INFO(this->get_logger(), "Received goal point (%.2f, %.2f), waiting for navigation server...", msg.pose.position.x, msg.pose.position.y);
    nav_to_pose_client_->wait_for_action_server();
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = msg;
    goal_msg.behavior_tree = "";
    available_ = false;
    RCLCPP_INFO(this->get_logger(), "Send goal point: (%.2f, %.2f)", goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);
    send_goal_future_ = nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options_);
}

void Navigator::nav_cancel() {
    nav_to_pose_client_->async_cancel_all_goals();
}

void Navigator::get_current_pose() {
    geometry_msgs::msg::TransformStamped bfMsg;
    try {
        bfMsg = tf2_buffer_->lookupTransform(
            "map", "base_footprint",
            tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform : %s",
            ex.what());
        return;
    }
    RMDecision::PoseStamped currentPose;
    currentPose.header.stamp = this->now();
    currentPose.header.frame_id = "map";
    currentPose.pose.position.x = bfMsg.transform.translation.x;
    currentPose.pose.position.y = bfMsg.transform.translation.y;
    currentPose.pose.position.z = bfMsg.transform.translation.z;
    currentPose.pose.orientation = bfMsg.transform.rotation;

    current_pose_pub_->publish(currentPose);
}