#include "weak_solution/weak_solution.hpp"
#include <random>
using namespace RMDecision;
using namespace std::chrono_literals;

WeakSolution::WeakSolution(const rclcpp::NodeOptions& options) : DecisionBase(7, "weak_solution", options) {
    timer_ = this->create_wall_timer(5s, std::bind(&WeakSolution::timer_callback, this));
}

void WeakSolution::timer_callback() {
    PlaneCoordinate nextPoint = get_random_point(1.5);
    to_target_point(nextPoint);
}

PlaneCoordinate WeakSolution::get_random_point(double radius) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-radius, radius);
    auto tmp_point = PlaneCoordinate(
        prism_.self.pose.pose.position.x, prism_.self.pose.pose.position.y);
    tmp_point = tmp_point + PlaneCoordinate(dis(gen), dis(gen));
    return tmp_point;
}

void WeakSolution::to_target_point(const PlaneCoordinate& point) {
    nav_to_pose(point.to_pose_stamped(this->now()), false);
}

void WeakSolution::to_target_point(double x, double y) {
    nav_to_pose(PlaneCoordinate(x, y).to_pose_stamped(this->now()), false);
}
