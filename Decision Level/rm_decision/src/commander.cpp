#include <functional>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>


#include <memory>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <string>
#include <unistd.h>
#include <vector>
#include <fstream>
#include <cmath>
#include <bitset>
#include <chrono>


#include "rm_decision/commander.hpp"

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>

using namespace std::chrono_literals;

namespace rm_decision {

    Commander::Commander(const rclcpp::NodeOptions &options) : Node("commander", options) {
        RCLCPP_INFO(this->get_logger(), "Commander node has been started.");

        //创建客户端
        nav_to_pose_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
        send_goal_options.goal_response_callback = std::bind(&Commander::goal_response_callback, this,
                                                             std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&Commander::feedback_callback, this, std::placeholders::_1,
                                                        std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&Commander::result_callback, this, std::placeholders::_1);
        //初始化状态
        loadNavPoints();
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
        sleep(2);
        //初始化策略 默认是1号策略
        this->declare_parameter<int>("strategy", 1);
        this->get_parameter("strategy", strategy);

        this->declare_parameter<int>("sentry_want_shangpo", 2);
        this->get_parameter("sentry_want_shangpo", sentry_want_shangpo);
        this->declare_parameter<int>("addhp_threshold", 100);
        this->get_parameter("addhp_threshold", addhp_threshold);
        this->declare_parameter<int>("addhp_full_threshold", 400);
        this->get_parameter("addhp_full_threshold", addhp_full_threshold);
        this->declare_parameter<int>("defend_threshold", 3000);
        this->get_parameter("defend_threshold", defend_threshold);
        this->declare_parameter<int>("buy_to_relive_goldcoin_threshold", 400);
        this->get_parameter("buy_to_relive_goldcoin_threshold", buy_to_relive_goldcoin_threshold);
        this->declare_parameter<int>("buy_ammo_remotely_ammo_threshold_when_wudi", 50);
        this->get_parameter("buy_ammo_remotely_ammo_threshold_when_wudi", buy_ammo_remotely_ammo_threshold_when_wudi);
        this->declare_parameter<int>("buy_ammo_remotely_ammo_threshold_when_youdi", 50);
        this->get_parameter("buy_ammo_remotely_ammo_threshold_when_youdi", buy_ammo_remotely_ammo_threshold_when_youdi);
        this->declare_parameter<int>("buy_ammo_remotely_hp_threshold_when_youdi", 50);
        this->get_parameter("buy_ammo_remotely_hp_threshold_when_youdi", buy_ammo_remotely_hp_threshold_when_youdi);
        this->declare_parameter<int>("buy_ammo_remotely_goldcoin_threshold_when_wudi", 400);
        this->get_parameter("buy_ammo_remotely_goldcoin_threshold_when_wudi",
                            buy_ammo_remotely_goldcoin_threshold_when_wudi);
        this->declare_parameter<int>("buy_ammo_remotely_goldcoin_threshold_when_youdi", 400);
        this->get_parameter("buy_ammo_remotely_goldcoin_threshold_when_youdi",
                            buy_ammo_remotely_goldcoin_threshold_when_youdi);
        this->declare_parameter<int>("buy_hp_remotely_hp_threshold", 200);
        this->get_parameter("buy_hp_remotely_hp_threshold", buy_hp_remotely_hp_threshold);
        this->declare_parameter<int>("buy_hp_remotely_goldcoin_threshold", 300);
        this->get_parameter("buy_hp_remotely_goldcoin_threshold", buy_hp_remotely_goldcoin_threshold);
        this->declare_parameter<int>("buy_ammo_local_ammo_threshold", 50);
        this->get_parameter("buy_ammo_local_ammo_threshold", buy_ammo_local_ammo_threshold);
        this->declare_parameter<int>("buy_ammo_local_goldcoin_threshold", 200);
        this->get_parameter("buy_ammo_local_goldcoin_threshold", buy_ammo_local_goldcoin_threshold);
        this->declare_parameter<int>("self_outpose_threshold", 500);
        this->get_parameter("self_outpose_threshold", self_outpose_threshold);
        this->declare_parameter<int>("buy_ammo_num_at_a_time", 100);
        this->get_parameter("buy_ammo_num_at_a_time", buy_ammo_num_at_a_time);
        this->declare_parameter<int>("not_buy_but_relive", 1);
        this->get_parameter("not_buy_but_relive", not_buy_but_relive);
        this->declare_parameter<int>("time_to_stop_enginner", 5);
        this->get_parameter("time_to_stop_enginner", time_to_stop_enginner);
        this->declare_parameter<int>("addhp_timeout", 20);
        this->get_parameter("addhp_timeout", addhp_timeout);
        this->declare_parameter<int>("shangpo_timeout", 20);
        this->get_parameter("shangpo_timeout", shangpo_timeout);
        this->declare_parameter<int>("if_open_movestats", 2);
        this->get_parameter("if_open_movestats", if_open_movestats);


        if (sentry_want_shangpo == 2) {
            shangpofail = true;
        }
        RCLCPP_INFO(this->get_logger(), "开始");
        currentState = std::make_shared<WaitState>(this);

        nav_sub_ = this->create_subscription<rm_decision_interfaces::msg::FromSerial>(
                "fromjudge", 10, std::bind(&Commander::serial_callback, this, std::placeholders::_1));
        aim_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
                "/tracker/target", rclcpp::SensorDataQoS(),
                std::bind(&Commander::aim_callback, this, std::placeholders::_1));
        enemypose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                "/tracker/enemypose", 10, std::bind(&Commander::enemypose_callback, this, std::placeholders::_1));
        sentry_cmd_pub_ = this->create_publisher<rm_decision_interfaces::msg::ToSerial>("sentry/cmd", 10);
        sentry_status_pub_ = this->create_publisher<rm_decision_interfaces::msg::ToAutoAim>("sentry/status", 10);

        // 创建线程（处理信息和发布命令）
        commander_thread_ = std::thread(&Commander::decision, this);
        executor_thread_ = std::thread(&Commander::executor, this);
    }

    // 析构函数
    Commander::~Commander() {
        if (commander_thread_.joinable()) {
            commander_thread_.join();
        }

        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
    }


    void Commander::decision() {
        rclcpp::Rate r(0.5);
        // behavetree init
        BT::BehaviorTreeFactory factory;
        factory.registerSimpleCondition("IfAddHp", std::bind(&Commander::IfAddHp, this));
        factory.registerSimpleCondition("IfAsked", std::bind(&Commander::IfAsked, this));
        factory.registerSimpleCondition("IfAttack", std::bind(&Commander::IfAttack, this));
        factory.registerSimpleCondition("IfBuyAmmo", std::bind(&Commander::IfBuyAmmo, this));
        factory.registerSimpleCondition("IfBuyAmmoRemotely", std::bind(&Commander::IfBuyAmmoRemotely, this));
        factory.registerSimpleCondition("IfBuyHp", std::bind(&Commander::IfBuyHp, this));
        factory.registerSimpleCondition("IfBuyToRelive", std::bind(&Commander::IfBuyToRelive, this));
        factory.registerSimpleCondition("IfDefend", std::bind(&Commander::IfDefend, this));
        factory.registerSimpleCondition("IfGoToEnemyOutpose", std::bind(&Commander::IfGoToEnemyOutpose, this));
        factory.registerSimpleCondition("IfGoToStopEngineer", std::bind(&Commander::IfGoToStopEngineer, this));
        factory.registerSimpleCondition("IfGoToStopHero", std::bind(&Commander::IfGoToStopHero, this));
        factory.registerSimpleCondition("IfGuard", std::bind(&Commander::IfGuard, this));
        factory.registerSimpleCondition("IfOutposeAlive", std::bind(&Commander::IfOutposeAlive, this));
        factory.registerSimpleCondition("S1", std::bind(&Commander::S1, this));
        factory.registerSimpleCondition("S2", std::bind(&Commander::S2, this));
        factory.registerSimpleCondition("S3", std::bind(&Commander::S3, this));
        factory.registerSimpleCondition("wait_for_start", std::bind(&Commander::wait_for_start, this));
        factory.registerSimpleCondition("IfDefendOutpose", std::bind(&Commander::IfDefendOutpose, this));

        factory.registerSimpleAction("BuyAmmoRemotely_handle", std::bind(&Commander::BuyAmmoRemotely_handle, this));
        factory.registerSimpleAction("BuyAmmo_handle", std::bind(&Commander::BuyAmmo_handle, this));
        factory.registerSimpleAction("BuyHp_handle", std::bind(&Commander::BuyHp_handle, this));
        factory.registerSimpleAction("BuyToRelive_handle", std::bind(&Commander::BuyToRelive_handle, this));
        factory.registerSimpleAction("Gimbal_handle", std::bind(&Commander::Gimbal_handle, this));
        factory.registerSimpleAction("GoToEnemyOutpose_handle", std::bind(&Commander::GoToEnemyOutpose_handle, this));
        factory.registerSimpleAction("GoToStopEngineer_handle", std::bind(&Commander::GoToStopEngineer_handle, this));
        factory.registerSimpleAction("GoToStopHero_handle", std::bind(&Commander::GoToStopHero_handle, this));
        factory.registerSimpleAction("Guard", std::bind(&Commander::Guard, this));
        factory.registerSimpleAction("S2GoToOutpose", std::bind(&Commander::S2GoToOutpose, this));
        factory.registerSimpleAction("S3Patro", std::bind(&Commander::S3Patro, this));
        factory.registerSimpleAction("addhp_handle", std::bind(&Commander::addhp_handle, this));
        factory.registerSimpleAction("attack_handle", std::bind(&Commander::attack_handle, this));
        factory.registerSimpleAction("defend_handle", std::bind(&Commander::defend_handle, this));
        factory.registerSimpleAction("S2DefendOutpose", std::bind(&Commander::S2DefendOutpose, this));

        auto tree = factory.createTreeFromFile("./src/rm_decision/rm_decision/config/sentry_bt.xml"); //official
        // auto tree = factory.createTreeFromFile("./rm_decision/config/sentry_bt.xml");  //for debug
        BT::Groot2Publisher publisher(tree);
        while (rclcpp::ok()) {
            tree.tickWhileRunning();
            //RCLCPP_INFO(this->get_logger(), "自身血量: %f, 自身弹量: %f, 自身金币: %f color: %d, gamestary: %d, enemy_outpose: %d, self_outpose:%f",self_hp,self_ammo,goldcoin,color,gamestart,enemy_outpost_hp,self_outpost);

            rm_decision_interfaces::msg::ToAutoAim msg;
            msg.target = sentry_status;
            sentry_status_pub_->publish(msg);
            r.sleep();
        }

    }


    // 执行器线程
    void Commander::executor() {
        rclcpp::Rate r(5);
        while (rclcpp::ok()) {
            getcurrentpose();
            currentState->handle();
            if (sentry_want_shangpo == 1) {
                checkpo();
                canshangpo();
            }
            if (state != change_state) {
                RCLCPP_INFO(this->get_logger(), "状态改变");
                checkgoal = true;
                state = change_state;
            }
            r.sleep();
        }
    }


    // 改变状态
    void Commander::setState(std::shared_ptr<State> state) {
        if (currentState != state && if_open_movestats == 1) {
            move_points_.clear();
        }
        currentState = state;
    }


    // 导航到点
    void Commander::nav_to_pose(geometry_msgs::msg::PoseStamped goal_pose) {
        nav_to_pose_client->wait_for_action_server();
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = goal_pose;
        goal_msg.behavior_tree = "";
        send_goal_future = nav_to_pose_client->async_send_goal(goal_msg, send_goal_options);
    }

    // 请求反馈
    void Commander::goal_response_callback(
            rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal was rejected by server");
            checkgoal = true;
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    // 过程反馈
    void Commander::feedback_callback(
            rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future,
            const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback
    ) {
        // RCLCPP_INFO(this->get_logger(),"Received feedback: 去往目标点的距离: %.2f m",feedback->distance_remaining);
    }

    // 结果反馈
    void Commander::result_callback(
            const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was reached!");
                nav_state = 1;
                checkgoal = true;
                failed_count = 0;
                endtime = std::chrono::steady_clock::now();
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(), "Goal was aborted");
                nav_state = 2;
                checkgoal = true;
                failed_count++;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                nav_state = 3;
                checkgoal = true;
                failed_count++;
                break;
            default:
                RCLCPP_INFO(get_logger(), "Unknown result code");
                nav_state = 4;
                checkgoal = true;
                failed_count++;
                break;
        }
    }

    // 读取导航点
    void Commander::processPoses(std::vector<double> &pose_param,
                                 std::vector<geometry_msgs::msg::PoseStamped> &nav_points_) {
        for (uint i = 0; i < pose_param.size(); i = i + 3) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = pose_param[i];
            pose.pose.position.y = pose_param[i + 1];
            pose.pose.position.z = pose_param[i + 2];
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            nav_points_.push_back(pose);
            RCLCPP_INFO(this->get_logger(), "传入第%d个导航点: %.2f, %.2f, %.2f", (i + 3) / 3, pose.pose.position.x,
                        pose.pose.position.y, pose.pose.position.z);
        }
    }

    // 加载导航点
    void Commander::loadNavPoints() {
        RCLCPP_INFO(this->get_logger(), "开始传入导航点");
        geometry_msgs::msg::PoseStamped pose;
        std::vector<double> pose_list;
        std::vector<std::string> route_list = {"Guard_points", "self_addhp_point", "self_base_point",
                                               "S1_Stop_Engineer_point", "S1_Stop_Hero_points", "S1_Outpost_point",
                                               "S2_Outpose_point", "S2_Defend_point",
                                               "S3_Patro_points",
                                               "Guard_points2", "Guard_points3"};

        std::vector<std::string> area_list = {"po_area1", "po_area2", "po_area3"};
        std::vector<std::vector<geometry_msgs::msg::PoseStamped>>::iterator list = list_name.begin();
        std::vector<std::vector<geometry_msgs::msg::PoseStamped>>::iterator area = po_name.begin();


        for (auto it = route_list.begin(); it != route_list.end(); it++) {
            this->declare_parameter(*it, pose_list);
            auto pose_param = this->get_parameter(*it).as_double_array();
            processPoses(pose_param, *list);
            RCLCPP_INFO(this->get_logger(), "%s随机导航点个数: %ld", it->c_str(), (*list).size());
            list++;
        }
        for (auto it = area_list.begin(); it != area_list.end(); it++) {
            this->declare_parameter(*it, pose_list);
            auto pose_param = this->get_parameter(*it).as_double_array();
            processPoses(pose_param, *area);
            RCLCPP_INFO(this->get_logger(), "%s随机导航点个数: %ld", it->c_str(), (*area).size());
            area++;
        }
        Guard_points = list_name.at(0);
        self_addhp_point = list_name.at(1);
        self_base_point = list_name.at(2);
        S1_Stop_Engineer_point = list_name.at(3);
        S1_Stop_Hero_points = list_name.at(4);
        S1_Outpost_point = list_name.at(5);
        S2_Outpose_point = list_name.at(6);
        S2_Defend_point = list_name.at(7);
        S3_Patro_points = list_name.at(8);
        Guard_points2 = list_name.at(9);
        Guard_points3 = list_name.at(10);
    }

    void Commander::checkpo() {
        std::vector<std::vector<geometry_msgs::msg::PoseStamped>>::iterator area;
        uint i = 0;
        for (i = 0; i != po_name.size(); i++) {
            if (isinpo(po_name[i], currentpose)) {
                checkpo_shangpoing = true;
                diffyaw = getyawdiff(po_name[i][0], po_name[i][1]);
                // RCLCPP_INFO(this->get_logger(), "在坡 diffyaw=%f", diffyaw);
                break;
            }
        }
        if (i == po_name.size()) {
            checkpo_shangpoing = false;
            diffyaw = 0;
            // RCLCPP_INFO(this->get_logger(), "no po");
        }

    }

    // 加载敌军hp
    uint Commander::enemyhp() {
        switch (enemy_num) {
            case 1:
                return enemy_1.hp;
            case 2:
                return enemy_2.hp;
            case 3:
                return enemy_3.hp;
            case 4:
                return enemy_4.hp;
            case 5:
                return enemy_5.hp;
        }
        return 600;
    }


    // 订阅回调
    void Commander::serial_callback(const rm_decision_interfaces::msg::FromSerial::SharedPtr msg) {
        gamestart = msg->gamestart;
        color = msg->color;
        self_ammo = msg->projectile_allowance_17mm;
        goldcoin = msg->remaining_gold_coin;
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
        if (init) {
            call_goal.pose.position.x = msg->target_pos_x;
            call_goal.pose.position.y = msg->target_pos_y;
            init = false;
        }
        if (msg->target_pos_x != call_goal.pose.position.x || msg->target_pos_y != call_goal.pose.position.y) {
            call_goal.pose.position.x = msg->target_pos_x;
            call_goal.pose.position.y = msg->target_pos_y;
            is_called = true;
        }
    }

    void Commander::aim_callback(const auto_aim_interfaces::msg::Target::SharedPtr msg) {
        //RCLCPP_INFO(this->get_logger(), "自瞄回调");
        tracking = msg->tracking;
        enemy_num = msg->armors_num;
    }

    void Commander::enemypose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        //RCLCPP_INFO(this->get_logger(), "自瞄回调");
        enemypose.header.stamp = this->now();
        enemypose.header.frame_id = "map";
        enemypose.pose.position.x = msg->point.x;
        enemypose.pose.position.y = msg->point.y;
        enemypose.pose.position.z = msg->point.z;
        RCLCPP_INFO(this->get_logger(), "敌方位置: %.2f, %.2f, %.2f", enemypose.pose.position.x,
                    enemypose.pose.position.y, enemypose.pose.position.z);
    }

    //取距
    double Commander::distence(const geometry_msgs::msg::PoseStamped a) {
        double dis = sqrt(pow(a.pose.position.x, 2) + pow(a.pose.position.y, 2));
        return dis;
    }

    void Commander::canshangpo() {
        if (!shangpofail && !shanpotimer) {
            start_time = std::chrono::steady_clock::now();
            shanpotimer = true;
        }
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count() >= shangpo_timeout) {
            if (checkpo_shangpoing) {
                shangpofail = true;
            }
            shanpotimer = false;
        }
    }

    bool Commander::isinpo(std::vector<geometry_msgs::msg::PoseStamped> area, geometry_msgs::msg::PoseStamped goal) {
        uint i, j;
        bool c = false;
        for (i = 0, j = area.size() - 1; i < area.size(); j = i++) {
            if (((area[i].pose.position.y > goal.pose.position.y) !=
                 (area[j].pose.position.y > goal.pose.position.y)) &&
                (goal.pose.position.x < (area[j].pose.position.x - area[i].pose.position.x) *
                                        (goal.pose.position.y - area[i].pose.position.y) /
                                        (area[j].pose.position.y - area[i].pose.position.y) + area[i].pose.position.x))
                c = !c;
        }
        return c;
    }

    float Commander::getyawdiff(geometry_msgs::msg::PoseStamped a, geometry_msgs::msg::PoseStamped b) {
        float diffyaw, goalyaw;
        goalyaw = atan((b.pose.position.y - a.pose.position.y) / (b.pose.position.x - a.pose.position.x));
        //    RCLCPP_INFO(this->get_logger(), "goal%f cur %f", goalyaw,currentpose.pose.orientation.z);
        diffyaw = (goalyaw - asin(currentpose.pose.orientation.z) * 2) * (180.0 / M_PI);
        diffyaw = fmod(diffyaw + 180, 360) - 180;
        return diffyaw;
    }


    // 巡逻模式
    void PatrolState::handle() {
        if (commander->checkgoal && std::chrono::steady_clock::now() - commander->endtime > 3s) {
            commander->nav_to_pose(*commander->random);
            commander->random++;
            if (commander->random == commander->Patro_points.end()) {
                commander->random = commander->Patro_points.begin();
            }
            commander->checkgoal = false;
        }
    }

    // goandstay模式(可用于守卫某点或逃跑等)
    void GoAndStayState::handle() {
        if (commander->checkgoal) {
            commander->nav_to_pose(commander->goal);
            commander->checkgoal = false;
        }
    }

    // 追击模式
    void AttackState::handle() {
        commander->nav_to_pose(commander->enemypose);
        RCLCPP_INFO(commander->get_logger(), "敌方距离: %.2f,开始追击", commander->distence(commander->enemypose));
    }

    // 等待模式
    void WaitState::handle() {

    }

    //振荡模式
    void MoveState::handle() {
        if (commander->move_points_.empty()) {
            commander->move_points_ = commander->generateRandomPoints(10, 1.5);
            commander->move = commander->move_points_.begin();
        }

        if (commander->checkgoal) {
            commander->nav_to_pose(*commander->move);
            commander->move++;
            if (commander->move == commander->move_points_.end()) {
                commander->move = commander->move_points_.begin();
            }
            commander->checkgoal = false;
        }
    }


    //联盟赛获取自身坐标
    void Commander::getcurrentpose() {
        geometry_msgs::msg::TransformStamped odom_msg;
        try {
            odom_msg = tf2_buffer_->lookupTransform(
                    "map", "livox_frame",
                    tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            // RCLCPP_INFO(
            //         this->get_logger(), "Could not transform : %s",
            //         ex.what());
            return;
        }
        currentpose.header.stamp = this->now();
        currentpose.header.frame_id = "map";
        currentpose.pose.position.x = odom_msg.transform.translation.x;
        currentpose.pose.position.y = odom_msg.transform.translation.y;
        currentpose.pose.position.z = odom_msg.transform.translation.z;
        currentpose.pose.orientation = odom_msg.transform.rotation;
        // RCLCPP_INFO(this->get_logger(), "当前位置: %.2f, %.2f, %.2f", currentpose.pose.position.x,
        //             currentpose.pose.position.y, currentpose.pose.position.z);
    }


    // 生成随机点
    std::vector<geometry_msgs::msg::PoseStamped> Commander::generateRandomPoints(int num_points, double radius) {
        getcurrentpose();
        std::vector<geometry_msgs::msg::PoseStamped> points;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-radius, radius);

        for (int i = 0; i < num_points; ++i) {
            geometry_msgs::msg::PoseStamped point;
            point.header.frame_id = "map";
            point.pose.position.x = currentpose.pose.position.x + dis(gen);
            point.pose.position.y = currentpose.pose.position.y + dis(gen);
            point.pose.position.z = currentpose.pose.position.z;
            point.pose.orientation = currentpose.pose.orientation;
            points.push_back(point);
        }

        return points;
    }

} // namespace rm_decision
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_decision::Commander)
