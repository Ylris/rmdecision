#include "sensing_unit/sensing_unit.hpp"

SensingUnit::SensingUnit(const rclcpp::NodeOptions& options) : Node("observe_unit", options), chessboard_(RMDecision::Faction::UNKNOWN) {
    std::string faction_str;
    this->declare_parameter<std::string>("faction");

    if (!this->get_parameter("faction", faction_str)) {
        RCLCPP_FATAL(this->get_logger(), "Faction is not defined. SensingUnit will shut down.");
        rclcpp::shutdown();
        return;
    }

    RMDecision::Faction faction;
    if (faction_str == "RED" || faction_str == "Red" || faction_str == "R" || faction_str == "red" || faction_str == "1") {
        faction = RMDecision::Faction::RED;
    } else if (faction_str == "BLUE" || faction_str == "Blue" || faction_str == "B" || faction_str == "blue" || faction_str == "2") {
        faction = RMDecision::Faction::BLUE;
    } else {
        faction = RMDecision::Faction::UNKNOWN;
    }
    init_chessboard(faction);

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto subOpt = rclcpp::SubscriptionOptions();
    auto pubOpt = rclcpp::PublisherOptions();
    subOpt.callback_group = callback_group_;
    pubOpt.callback_group = callback_group_;

    all_robot_hp_sub_ = this->create_subscription<rm_decision_interfaces::msg::AllRobotHP>(
        "all_robot_hp", 10, std::bind(&SensingUnit::all_robot_hp_callback, this, std::placeholders::_1), subOpt);

    friend_location_sub_ = this->create_subscription<rm_decision_interfaces::msg::FriendLocation>(
        "friend_location", 10, std::bind(&SensingUnit::friend_location_callback, this, std::placeholders::_1), subOpt);

    tracking_pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/tracker/enemypose", 10, std::bind(&SensingUnit::tracking_pose_callback, this, std::placeholders::_1), subOpt);

    from_serial_sub_ = this->create_subscription<rm_decision_interfaces::msg::FromSerial>(
        "fromjudge", 10, std::bind(&SensingUnit::from_serial_callback, this, std::placeholders::_1), subOpt);

    receive_serial_sub_ = this->create_subscription<rm_decision_interfaces::msg::ReceiveSerial>(
        "receive_serial", 10, std::bind(&SensingUnit::receive_serial_callback, this, std::placeholders::_1), subOpt);

    robot_status_sub_ = this->create_subscription<rm_decision_interfaces::msg::RobotStatus>(
        "robot_status", 10, std::bind(&SensingUnit::robot_status_callback, this, std::placeholders::_1), subOpt);

    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "/tracker/target", rclcpp::SensorDataQoS(), std::bind(&SensingUnit::target_callback, this, std::placeholders::_1), subOpt);

    current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/navigator/current_pose", 10, std::bind(&SensingUnit::current_pose_callback, this, std::placeholders::_1), subOpt);

    chessboard_pub_ = this->create_publisher<iw_interfaces::msg::Chessboard>("rm_decision/chessboard", 10, pubOpt);
    std::string prismPubTopicName = "rm_decision/prism/" + std::to_string(prism_.self.id);
    prism_pub_ = this->create_publisher<iw_interfaces::msg::Prism>(prismPubTopicName.c_str(), 10, pubOpt);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SensingUnit::timer_callback, this), callback_group_);
}

void SensingUnit::init_chessboard(const RMDecision::Faction& faction) {
    chessboard_.faction = faction;
    for (const auto& elem : RMDecision::DefaultInfo::robots) {
        (*chessboard_.robots)[elem.first] = std::make_shared<RMDecision::Robot>(elem.second);
    }
    init_map_declare<std::vector<double>, RMDecision::Terrain>(
        RMDecision::DefaultInfo::terrains, *chessboard_.terrains, RMDecision::Terrain::array_to_terrain);
    init_map_declare<std::vector<double>, RMDecision::Architecture>(
        RMDecision::DefaultInfo::architecture, *chessboard_.architectures, RMDecision::Architecture::array_to_architecture);

    name_objects<RMDecision::Robot>(*chessboard_.robots);
    name_objects<RMDecision::Terrain>(*chessboard_.terrains);
    name_objects<RMDecision::Architecture>(*chessboard_.architectures);
    if (chessboard_.faction != RMDecision::Faction::UNKNOWN) {
        chessboard_.initialed = true;
    }
}

void SensingUnit::timer_callback() {
    chessboard_pub_->publish(chessboard_.to_message());
    prism_pub_->publish(prism_.to_message());
}

void SensingUnit::all_robot_hp_callback(const rm_decision_interfaces::msg::AllRobotHP::SharedPtr msg) {
    if (chessboard_.initialed) {
        assert(msg->color == chessboard_.faction && "Faction Maching ERROR");
    } else {
        chessboard_.faction = static_cast<RMDecision::Faction>(msg->color);
        chessboard_.initialed = true;
    }

    auto& robots = *chessboard_.robots;
    robots["R1"]->hp = msg->red_1_robot_hp;
    robots["R2"]->hp = msg->red_2_robot_hp;
    robots["R3"]->hp = msg->red_3_robot_hp;
    robots["R4"]->hp = msg->red_4_robot_hp;
    robots["R5"]->hp = msg->red_5_robot_hp;
    robots["R7"]->hp = msg->red_7_robot_hp;

    robots["B1"]->hp = msg->blue_1_robot_hp;
    robots["B2"]->hp = msg->blue_2_robot_hp;
    robots["B3"]->hp = msg->blue_3_robot_hp;
    robots["B4"]->hp = msg->blue_4_robot_hp;
    robots["B5"]->hp = msg->blue_5_robot_hp;
    robots["B7"]->hp = msg->blue_7_robot_hp;

    auto& architectures = *chessboard_.architectures;
    architectures["Red_Outpost"]->hp = msg->red_outpost_hp;
    architectures["Red_Base"]->hp = msg->red_base_hp;
    architectures["Blue_Outpost"]->hp = msg->blue_outpost_hp;
    architectures["Blue_Base"]->hp = msg->blue_base_hp;

    chessboard_.timestamp = this->now();

    prism_.self.hp = chessboard_.friend_robot(prism_.self.id)->hp;
    prism_.track.hp = chessboard_.enemy_robot(prism_.track.id)->hp;
}

void SensingUnit::friend_location_callback(const rm_decision_interfaces::msg::FriendLocation::SharedPtr msg) {
    auto lambda = [&](uint id, double x, double y) {
        chessboard_.friend_robot(id)->pose = RMDecision::PlaneCoordinate(x, y).to_pose_stamped(this->now());
    };

    lambda(1, msg->hero_x, msg->hero_y);
    lambda(2, msg->engineer_x, msg->engineer_y);
    lambda(3, msg->standard_3_x, msg->standard_3_y);
    lambda(4, msg->standard_4_x, msg->standard_4_y);
    lambda(5, msg->standard_5_x, msg->standard_5_y);

    chessboard_.timestamp = this->now();
}

void SensingUnit::from_serial_callback(const rm_decision_interfaces::msg::FromSerial::SharedPtr msg) {
    if (chessboard_.initialed) {
        assert(msg->color == chessboard_.faction && "Faction Maching ERROR");
    } else {
        chessboard_.faction = static_cast<RMDecision::Faction>(msg->color);
        chessboard_.initialed = true;
    }

    uint self_hp, self_base, self_outpost, enemy_outpost_hp;

    if (msg->color == 1) {
        self_hp = msg->red_7;
        self_base = msg->red_base_hp;
        self_outpost = msg->red_outpost_hp;
        enemy_outpost_hp = msg->blue_outpost_hp;
    } else {
        self_hp = msg->blue_7;
        self_base = msg->blue_base_hp;
        self_outpost = msg->blue_outpost_hp;
        enemy_outpost_hp = msg->red_outpost_hp;
    }

    prism_.self.hp = self_hp;
    prism_.game.game_start = msg->gamestart;
    prism_.game.coins = msg->remaining_gold_coin;
    prism_.game.projectile_allowance = msg->projectile_allowance_17mm;
    prism_.call.target = RMDecision::PlaneCoordinate(msg->target_pos_x, msg->target_pos_y);
    prism_.call.is_called = true;

    chessboard_.friend_robot(prism_.self.id)->hp = self_hp;
    chessboard_.friend_base()->hp = self_base;
    chessboard_.friend_outpost()->hp = self_outpost;
    chessboard_.enemy_outpost()->hp = enemy_outpost_hp;

    chessboard_.timestamp = this->now();
}

void SensingUnit::tracking_pose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    prism_.track.pose.header.stamp = this->now();
    prism_.track.pose.header.frame_id = "map";
    prism_.track.pose.pose.position.x = msg->point.x;
    prism_.track.pose.pose.position.y = msg->point.y;
    prism_.track.pose.pose.position.z = msg->point.z;
}

void SensingUnit::target_callback(const auto_aim_interfaces::msg::Target::SharedPtr msg) {
    prism_.track.id = msg->armors_num;
    prism_.track.tracking = msg->tracking;
}

void SensingUnit::receive_serial_callback(const rm_decision_interfaces::msg::ReceiveSerial::SharedPtr msg) {
    if (chessboard_.initialed) {
        assert(msg->color == chessboard_.faction && "Faction Maching ERROR");
    } else {
        chessboard_.faction = static_cast<RMDecision::Faction>(msg->color);
        chessboard_.initialed = true;
    }

    auto& robots = *chessboard_.robots;
    robots["R1"]->hp = msg->red_1;
    robots["R2"]->hp = msg->red_2;
    robots["R3"]->hp = msg->red_3;
    robots["R4"]->hp = msg->red_4;
    robots["R5"]->hp = msg->red_5;
    robots["R7"]->hp = msg->red_7;

    robots["B1"]->hp = msg->blue_1;
    robots["B2"]->hp = msg->blue_2;
    robots["B3"]->hp = msg->blue_3;
    robots["B4"]->hp = msg->blue_4;
    robots["B5"]->hp = msg->blue_5;
    robots["B7"]->hp = msg->blue_7;

    auto& architectures = *chessboard_.architectures;
    architectures["Red_Outpost"]->hp = msg->red_outpost_hp;
    architectures["Red_Base"]->hp = msg->red_base_hp;
    architectures["Blue_Outpost"]->hp = msg->blue_outpost_hp;
    architectures["Blue_Base"]->hp = msg->blue_base_hp;

    chessboard_.timestamp = this->now();

    prism_.self.hp = msg->self_hp;
    prism_.track.hp = chessboard_.enemy_robot(prism_.track.id)->hp;
}

void SensingUnit::robot_status_callback(const rm_decision_interfaces::msg::RobotStatus::SharedPtr msg) {
    if (prism_.self.id == msg->robot_id) {
        prism_.self.hp = msg->current_hp;
        prism_.self.ammo = msg->ammo_buy;
        prism_.self.shooter_heat = msg->shooter_heat;
    }
}

void SensingUnit::current_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    prism_.self.pose = *msg;
    chessboard_.friend_robot(prism_.self.id)->pose = *msg;
    chessboard_.timestamp = this->now();
}
