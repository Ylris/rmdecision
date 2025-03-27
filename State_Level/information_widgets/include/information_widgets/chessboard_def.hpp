#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "iw_interfaces/msg/chessboard.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_decision_defs.hpp"

namespace RMDecision {

class ChessboardHandle {
public:
    ChessboardHandle(Faction faction) : faction(faction) {
        robots = std::make_shared<std::unordered_map<std::string, std::shared_ptr<Robot>>>();
        terrains = std::make_shared<std::unordered_map<std::string, std::shared_ptr<Terrain>>>();
        architectures = std::make_shared<std::unordered_map<std::string, std::shared_ptr<Architecture>>>();
        initialed = false;
        timestamp = rclcpp::Time(-1, 0, RCL_ROS_TIME);
    }

    ChessboardHandle(const iw_interfaces::msg::Chessboard& msg);
    // 使用智能指针避免对象切割
    std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<Robot>>> robots;
    std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<Terrain>>> terrains;
    std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<Architecture>>> architectures;
    Faction faction;
    rclcpp::Time timestamp;
    bool initialed = false;

    std::shared_ptr<Robot> friend_robot(const uint& id);

    std::shared_ptr<Robot> enemy_robot(const uint& id);

    std::shared_ptr<Architecture> friend_outpost();

    std::shared_ptr<Architecture> enemy_outpost();

    std::shared_ptr<Architecture> friend_base();

    std::shared_ptr<Architecture> enemy_base();

    void update_from_message(const iw_interfaces::msg::Chessboard& msg);

    iw_interfaces::msg::Chessboard to_message();
};

}  // namespace RMDecision