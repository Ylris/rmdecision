#include "information_widgets/prism_def.hpp"

RMDecision::Prism::Prism(const iw_interfaces::msg::Prism& msg) {
    update_from_message(msg);
}

void RMDecision::Prism::update_from_message(const iw_interfaces::msg::Prism& msg) {
    self.id = msg.self_id;
    self.hp = msg.self_hp;
    self.ammo = msg.self_ammo;
    self.shooter_heat = msg.self_shooter_heat;
    self.pose = msg.self_pose;

    track.tracking = msg.track_tracking;
    track.id = msg.track_id;
    track.hp = msg.track_hp;
    track.pose = msg.track_pose;

    game.game_start = msg.game_game_start;
    game.buy_ammo_ordered = msg.game_buy_ammo_ordered;
    game.coins = msg.game_coins;
    game.projectile_allowance = msg.game_projectile_allowance;

    call.target.x = msg.call_target_x;
    call.target.y = msg.call_target_y;
    call.is_called = msg.call_is_called;
}

iw_interfaces::msg::Prism RMDecision::Prism::to_message() {
    iw_interfaces::msg::Prism msg;
    msg.self_id = self.id;
    msg.self_hp = self.hp;
    msg.self_ammo = self.ammo;
    msg.self_shooter_heat = self.shooter_heat;
    msg.self_pose = self.pose;

    msg.track_tracking = track.tracking;
    msg.track_id = track.id;
    msg.track_hp = track.hp;
    msg.track_pose = track.pose;

    msg.game_game_start = game.game_start;
    msg.game_buy_ammo_ordered = game.buy_ammo_ordered;
    msg.game_coins = game.coins;
    msg.game_projectile_allowance = game.projectile_allowance;

    return msg;
}