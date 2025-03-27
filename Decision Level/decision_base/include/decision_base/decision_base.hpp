#pragma once

#include "information_widgets/chessboard_def.hpp"
#include "information_widgets/prism_def.hpp"
#include "information_widgets/rm_decision_defs.hpp"
#include "iw_interfaces/msg/chessboard.hpp"
#include "iw_interfaces/msg/prism.hpp"
#include "navigator_interfaces/msg/navigate.hpp"
#include "rclcpp/rclcpp.hpp"

namespace RMDecision {
class DecisionBase : public rclcpp::Node {
public:
    explicit DecisionBase(uint selfId, std::string nodeName, const rclcpp::NodeOptions& options);

protected:
    void nav_to_pose(const PoseStamped& stampedPose, bool instant);
    ChessboardHandle chessboard_;
    Prism prism_;

private:
    rclcpp::Subscription<iw_interfaces::msg::Chessboard>::SharedPtr chessboard_sub_;
    rclcpp::Subscription<iw_interfaces::msg::Prism>::SharedPtr prism_sub_;
    rclcpp::Publisher<navigator_interfaces::msg::Navigate>::SharedPtr nav_pub_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    void chessboard_sub_callback(const iw_interfaces::msg::Chessboard::SharedPtr msg);

    void prism_sub_callback(const iw_interfaces::msg::Prism::SharedPtr msg);
};

}  // namespace RMDecision
