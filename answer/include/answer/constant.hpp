#ifndef _CONSTANT_HPP_
#define _CONSTANT_HPP_

#include <string>

#include "serial.hpp"

namespace topic_name{
    const char* map = "map_topic";
    const char* area = "area";
    const char* robot = "robot";
    const char* pose = "pose";
    const char* shoot = "shoot";
    const char* password = "password";
    const char* password_segment = "password_segment";
}

namespace constant{
    const int grid_num_v = 256; // 水平方向网格数量
    const int grid_num_h = 128; // 垂直方向网格数量
    const int attack_distance = 80; // 攻击距离,敌人在攻击距离内再攻击
    const double danger_hp = 0.9; // 生命值低于这个值就要补充
    const int danger_bullet_num = 5; // 子弹低于这个值就要补充
    const double speed_scale = 1.0 / 32.0; // 速度缩放比例
    const std::string serial_path = "/dev/pts/2";
    const speed_t baud_rate = 115200;
}

#endif // ^^ !_CONSTANT_HPP_