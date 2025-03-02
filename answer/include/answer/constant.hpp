#ifndef _CONSTANT_HPP_
#define _CONSTANT_HPP_

#include <string>

namespace topic_name{
    const char* map = "map_topic";
    const char* area = "area";
    const char* robot = "robot";
    const char* pose = "pose";
    const char* shoot = "shoot";
    const char* password = "password";
    const char* password_segment = "password_segment";
    const char* restart = "restart";
}

namespace constant{
    constexpr int grid_num_v = 256; // 水平方向网格数量
    constexpr int grid_num_h = 128; // 垂直方向网格数量
    constexpr int attack_distance = 80; // 攻击距离,敌人在攻击距离内再攻击
    constexpr double danger_hp = 0.9; // 生命值低于这个值就要补充
    constexpr int danger_bullet_num = 5; // 子弹低于这个值就要补充
    constexpr double speed_scale = 1.0 / 32.0; // 速度缩放比例
    const std::string serial_path = "/dev/pts/2";
    constexpr int baud_rate = 115200;
}

namespace debug_option{
    constexpr bool print_our_pose = false;
    constexpr bool print_area_info = false;
    constexpr bool print_robot_info = false;
}

#endif // ^^ !_CONSTANT_HPP_