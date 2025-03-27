#pragma once

#include "tf2/LinearMath/Quaternion.h"

#include <cassert>
#include <cmath>
#include <string>
#include <vector>
#include <random>

#include "geometry_msgs/msg/pose.hpp"
#include "iw_interfaces/msg/architecture.hpp"
#include "iw_interfaces/msg/area.hpp"
#include "iw_interfaces/msg/plane_coordinate.hpp"
#include "iw_interfaces/msg/robot.hpp"
#include "iw_interfaces/msg/terrain.hpp"
#include "rclcpp/rclcpp.hpp"

namespace RMDecision {

const double INF = 1e9;

enum Faction { UNKNOWN,
               RED,
               BLUE };

typedef geometry_msgs::msg::PoseStamped PoseStamped;
typedef geometry_msgs::msg::Pose Pose;

class PlaneCoordinate {
public:
    double x;
    double y;

    PlaneCoordinate(const double& x = 0.0, const double& y = 0.0) : x(x), y(y) {}

    PlaneCoordinate(const PoseStamped& pose) : x(pose.pose.position.x), y(pose.pose.position.y) {}

    PlaneCoordinate(const iw_interfaces::msg::PlaneCoordinate msg) : x(msg.x), y(msg.y) {}

    PlaneCoordinate(const PlaneCoordinate& p1, const PlaneCoordinate& p2) {
        *this = p2 - p1;
    }

    inline PlaneCoordinate operator+(const PlaneCoordinate& another) const {
        return PlaneCoordinate(x + another.x, y + another.y);
    }

    inline void operator+=(const PlaneCoordinate& another) {
        x += another.x;
        y += another.y;
    }

    inline PlaneCoordinate operator-(const PlaneCoordinate& another) const {
        return PlaneCoordinate(x - another.x, y - another.y);
    }

    inline void operator-=(const PlaneCoordinate& another) {
        x -= another.x;
        y -= another.y;
    }

    inline PlaneCoordinate operator-() const {
        return PlaneCoordinate(-x, -y);
    }

    inline PlaneCoordinate operator*(const PlaneCoordinate& another) const {
        return PlaneCoordinate(x * another.x + y * another.y);
    }

    inline PlaneCoordinate operator*(const double& scale) const {
        return PlaneCoordinate(x * scale, y * scale);
    }

    inline void operator=(const PlaneCoordinate& another) {
        x = another.x;
        y = another.y;
    }

    inline bool operator==(const PlaneCoordinate& another) const {
        return x == another.x && y == another.y;
    }

    inline double norm() const {
        return sqrt(x * x + y * y);
    }

    inline double distance_to(const PlaneCoordinate& distination) const {
        return (*this - distination).norm();
    }

    PoseStamped to_pose_stamped(const rclcpp::Time& time) const {
        PoseStamped res;
        res.header.stamp = time;
        res.header.frame_id = "map";
        res.pose.position.x = x;
        res.pose.position.y = y;
        res.pose.position.z = 0;
        res.pose.orientation.x = 0;
        res.pose.orientation.y = 0;
        res.pose.orientation.z = 0;
        res.pose.orientation.w = 1;
        return res;
    }

    PoseStamped to_pose_stamped_with_orientation(
        const rclcpp::Time& time, const double orientation) const {
        PoseStamped res;
        res.header.stamp = time;
        res.header.frame_id = "map";
        res.pose.position.x = x;
        res.pose.position.y = y;
        res.pose.position.z = 0;
        tf2::Quaternion t;
        t.setRPY(0, 0, orientation);
        res.pose.orientation.x = t.x();
        res.pose.orientation.y = t.y();
        res.pose.orientation.z = t.z();
        res.pose.orientation.w = t.w();
        return res;
    }

    iw_interfaces::msg::PlaneCoordinate to_message() const {
        auto msg = iw_interfaces::msg::PlaneCoordinate();
        msg.x = x;
        msg.y = y;
        return msg;
    }

    static std::pair<double, double> line_across(const PlaneCoordinate& p1, const PlaneCoordinate& p2) {
        double k, b;
        k = (p1.y - p2.y) / (p1.x - p2.x);
        b = (p2.y * p1.x - p1.y * p2.x) / (p1.x - p2.x);
        return std::pair<double, double>(k, b);
    }

    inline bool parrallel_to(const PlaneCoordinate& another) const {
        return (another.x * y == another.y * x);
    }

    inline bool perp_to(const PlaneCoordinate& another) const {
        return (*this * another == 0);
    }

    // 判断由点p1, p2连接的线段和由点p3, p4连接的线段是否相交
    static bool intersect(const PlaneCoordinate& p1, const PlaneCoordinate& p2, const PlaneCoordinate& p3, const PlaneCoordinate& p4) {
        double x1 = p1.x, x2 = p2.x, x3 = p3.x, x4 = p4.x;
        double n1 = std::min(x1, x2), n2 = std::min(x3, x4);
        double m1 = std::max(x1, x2), m2 = std::max(x3, x4);
        double left, right;
        if (n1 > m2 || n2 > m1) {
            return false;
        }
        if (m1 - n2 >= m2 - n1) {
            left = n1;
            right = m2;
        } else {
            left = n2;
            right = m1;
        }
        double intersectionAbscissa = line_intersection_abscissa(p1, p2, p3, p4);
        return (intersectionAbscissa >= left && intersectionAbscissa <= right);
    }

    static double line_intersection_abscissa(const PlaneCoordinate& p1, const PlaneCoordinate& p2, const PlaneCoordinate& p3, const PlaneCoordinate& p4) {
        if (PlaneCoordinate(p1, p2).parrallel_to(PlaneCoordinate(p3, p4))) {
            return INF;
        }
        std::pair<double, double> coeff1 = line_across(p1, p2);
        std::pair<double, double> coeff2 = line_across(p3, p4);
        double k1 = coeff1.first;
        double k2 = coeff2.first;
        double b1 = coeff1.second;
        double b2 = coeff2.second;
        return -(b1 - b2) / (k1 - k2);
    }

    static PlaneCoordinate random_point(double radius) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-radius, radius);
        return PlaneCoordinate(dis(gen), dis(gen));
    }
};

class Object {
public:
    std::string label;

    Object() = default;

    Object(std::string label) : label(label) {}
};

class Robot : public Object {
public:
    Faction faction;
    uint id;
    PoseStamped pose;
    uint hp;
    uint level;
    bool attack = false;
    bool missing = true;

    Robot() = default;

    Robot(std::string label, Faction faction, int id, geometry_msgs::msg::PoseStamped pose, uint hp, uint level) : Object(label), faction(faction), id(id), pose(pose), hp(hp), level(level) {}

    Robot(const iw_interfaces::msg::Robot& msg)
        : Object(msg.label), id(msg.id), pose(msg.pose), hp(msg.hp), level(msg.level), attack(msg.attack), missing(msg.missing) {
        faction = static_cast<RMDecision::Faction>(msg.faction);
    }

    iw_interfaces::msg::Robot to_message() const {
        auto msg = iw_interfaces::msg::Robot();
        msg.attack = attack;
        msg.faction = faction;
        msg.hp = hp;
        msg.id = id;
        msg.label = label;
        msg.level = level;
        msg.missing = missing;
        msg.pose = pose;
        return msg;
    }

    void update_from_message(const iw_interfaces::msg::Robot& msg) {
        hp = msg.hp;
        level = msg.level;
        pose = msg.pose;
        attack = msg.attack;
        missing = msg.missing;
    }
};

class Area : public Object {
public:
    Area() {}

    Area(const iw_interfaces::msg::Area& msg) : Object(msg.label) {
        for (const auto& planeCoordinateMsg : msg.vertices) {
            vertices_.push_back(PlaneCoordinate(planeCoordinateMsg));
        }
    }

    Area(const std::string& label_, const std::vector<iw_interfaces::msg::PlaneCoordinate>& msgvertices_) : Object(label_) {
        for (const auto& planeCoordinateMsg : msgvertices_) {
            vertices_.push_back(PlaneCoordinate(planeCoordinateMsg));
        }
    }

    static void array_to_area(const std::vector<double>& doubleArray, Area& area) {
        assert((doubleArray.size() & 1) == 0 && "Extra Parameter ERROR");
        for (uint i = 0; i + 1 < doubleArray.size(); i += 2) {
            PlaneCoordinate coordinate;
            coordinate.x = doubleArray[i];
            coordinate.y = doubleArray[i + 1];
            area.vertices_.push_back(coordinate);
        }
    }

    iw_interfaces::msg::Area to_message() const {
        auto msg = iw_interfaces::msg::Area();
        for (const auto& vertex : vertices_) {
            auto planeCoordinateMsg = vertex.to_message();
            msg.vertices.push_back(planeCoordinateMsg);
        }
        msg.label = label;
        return msg;
    }

    bool contain(const PlaneCoordinate& point) const {
        auto infinity = PlaneCoordinate(point.x, INF);
        uint intersect_count = 0;
        for (size_t i = 0; i + 1 < vertices_.size(); i++) {
            if (PlaneCoordinate::intersect(point, infinity, vertices_[i], vertices_[i + 1])) {
                intersect_count++;
            }
        }
        if (PlaneCoordinate::intersect(point, infinity, vertices_[0], *vertices_.end())) {
            intersect_count++;
        }
        return (intersect_count % 2 == 1);
    }

protected:
    std::vector<PlaneCoordinate> vertices_;  // 按照逆/顺时针顺序初始化的顶点坐标
};

class Terrain : public Area {
    enum Type { UNDEFINED,
                BARRIER };

public:
    Terrain() {}

    Terrain(const iw_interfaces::msg::Terrain& msg) : Area(msg.label, msg.vertices) {}

    static void array_to_terrain(const std::vector<double>& doubleArray, Terrain& terrain) {
        assert((doubleArray.size() & 1) == 1 && "Extra Parameter ERROR");
        terrain.type_ = static_cast<Type>((int)doubleArray[0]);
        auto subDoubleArray = std::vector<double>(doubleArray.begin() + 1, doubleArray.end());
        array_to_area(subDoubleArray, terrain);
    }

    iw_interfaces::msg::Terrain to_message() const {
        auto msg = iw_interfaces::msg::Terrain();
        for (const auto& vertex : vertices_) {
            auto planeCoordinateMsg = vertex.to_message();
            msg.vertices.push_back(planeCoordinateMsg);
        }
        msg.label = label;
        msg.type = static_cast<int>(type_);
        return msg;
    }

    Type get_type() const {
        return type_;
    }

private:
    Type type_;
};

class Architecture : public Area {
public:
    uint hp;
    Faction faction;

    Architecture() {}

    Architecture(const iw_interfaces::msg::Architecture& msg) : Area(msg.label, msg.vertices), hp(msg.hp) {
        faction = static_cast<RMDecision::Faction>(msg.faction);
    }

    static void array_to_architecture(const std::vector<double>& doubleArray, Architecture& architecture) {
        array_to_architecture(doubleArray, architecture);
    }

    iw_interfaces::msg::Architecture to_message() const {
        auto msg = iw_interfaces::msg::Architecture();
        for (const auto& vertex : vertices_) {
            auto planeCoordinateMsg = vertex.to_message();
            msg.vertices.push_back(planeCoordinateMsg);
        }
        msg.label = label;
        msg.hp = hp;
        msg.faction = faction;
        return msg;
    }

    void update_from_message(const iw_interfaces::msg::Architecture msg) {
        hp = msg.hp;
    }

    // bool contain(PlaneCoordinate point) {

    // }
};

namespace DefaultInfo {
const std::unordered_map<std::string, RMDecision::Robot> robots = {
    {"R1", Robot("R1", RED, 1, PoseStamped(), 150, 1)},
    {"R2", Robot("R2", RED, 2, PoseStamped(), 150, 1)},
    {"R3", Robot("R3", RED, 3, PoseStamped(), 150, 1)},
    {"R4", Robot("R4", RED, 4, PoseStamped(), 150, 1)},
    {"R5", Robot("R5", RED, 5, PoseStamped(), 150, 1)},
    {"R7", Robot("R7", RED, 7, PoseStamped(), 600, 10)},
    {"B1", Robot("B1", BLUE, 1, PoseStamped(), 150, 1)},
    {"B2", Robot("B2", BLUE, 2, PoseStamped(), 150, 1)},
    {"B3", Robot("B3", BLUE, 3, PoseStamped(), 150, 1)},
    {"B4", Robot("B4", BLUE, 4, PoseStamped(), 150, 1)},
    {"B5", Robot("B5", BLUE, 5, PoseStamped(), 150, 1)},
    {"B7", Robot("B7", BLUE, 7, PoseStamped(), 600, 10)},
};

const std::unordered_map<std::string, std::vector<double>> terrains = {
    {"Guard_points", {}},
    {"self_addhp_point", {}},
    {"self_base_point", {}},
    {"S1_Stop_Engineer_point", {}},
    {"S1_Stop_Hero_points", {}},
    {"S1_Outpost_point", {}},
    {"S2_Outpose_point", {}},
    {"S2_Defend_point", {}},
    {"S3_Patro_points", {}},
    {"Guard_points2", {}},
    {"Guard_points3", {}},
    {"po_area1", {}},
    {"po_area2", {}},
    {"po_area3", {}},
};

const std::unordered_map<std::string, std::vector<double>> architecture = {
    {"Red_Outpost", {}},
    {"Red_Base", {}},
    {"Blue_Outpost", {}},
    {"Blue_Base", {}},
};

}  // namespace DefaultInfo

}  // namespace RMDecision