#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "geometry_msgs/msg/pose.hpp"
#include "ig_lio_c_msgs/srv/savereloc.hpp"
#include "information_widgets/rm_decision_defs.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "navigator_interfaces/msg/navigate.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

enum NavState {
    INIT,
    REACHED,
    ABORTED,
    CANCELED,
    UNKNOWN
};

class Navigator : public rclcpp::Node {
public:
    explicit Navigator(const rclcpp::NodeOptions& options);

private:
    rclcpp::Subscription<navigator_interfaces::msg::Navigate>::SharedPtr nav_msg_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
    std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> send_goal_future_;
    std::chrono::steady_clock::time_point endtime_;

    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf2_listener_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    rclcpp::Service<ig_lio_c_msgs::srv::Savereloc>::SharedPtr save_server_;

    NavState nav_state_;
    int failed_count_;
    bool available_;
    std::string label_;

    void tf2_listener_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

    void nav_callback(const navigator_interfaces::msg::Navigate::SharedPtr msg);

    void goal_response_callback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future);

    void feedback_callback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

    void result_callback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result);

    void timer_callback();

    void nav_to_pose(const geometry_msgs::msg::PoseStamped& msg);

    void nav_cancel();

    void get_current_pose();

    void save_callback(const ig_lio_c_msgs::srv::Savereloc::Request::SharedPtr request,
                       const ig_lio_c_msgs::srv::Savereloc::Response::SharedPtr response);
};