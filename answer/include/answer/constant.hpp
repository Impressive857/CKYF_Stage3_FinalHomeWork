#ifndef _CONSTANT_HPP_
#define _CONSTANT_HPP_

#include <string>

#include <opencv2/opencv.hpp>

namespace topic_name {
    const char* grid_map = "grid_map";
    const char* real_map = "real_map";
    const char* grid_area = "grid_area";
    const char* real_area = "real_area";
    const char* robot = "robot";
    const char* pose = "pose";
    const char* shoot = "shoot";
    const char* password = "password";
    const char* password_segment = "password_segment";
    const char* restart = "restart";
    const char* image_raw = "image_raw";
}

namespace constant {
    constexpr int grid_num_v = 256; // 水平方向网格数量
    constexpr int grid_num_h = 128; // 垂直方向网格数量
    constexpr int attack_distance = 400; // 攻击距离,敌人在攻击距离内再攻击
    constexpr int near_distance = 25; // 低于这个距离就视为很近
    constexpr int send_password_distance = 35; // 低于这个距离就视为到达密码发射区
    constexpr int attack_base_distance = 60; // 低于这个距离就认为到达基地
    constexpr double danger_hp = 0.9; // 生命值低于这个值就要补充
    constexpr double safe_hp = 1.0; // 生命值大于等于这个值就视为安全
    constexpr int danger_bullet_num = 5; // 子弹低于这个值就要补充
    constexpr double speed_scale = 1.0 / 128.0; // 速度缩放比例
    const std::string serial_path = "/dev/pts/2"; // 串口路径
    constexpr int baud_rate = 115200; // 波特率
    constexpr int enemy_max_x = 1900; // x大于这个值就视为基地旁边的敌人

    constexpr int gate_min_width = 20;  // 通道最小宽度
    constexpr int gate_max_width = 50;  // 通道最大宽度
    constexpr int gate_min_height = 20; // 通道最小高度
    constexpr int gate_max_height = 50; // 通道最大高度

    constexpr int recover_min_width = 40;  // 补给区最小宽度
    constexpr int recover_max_width = 60;  // 补给区最大宽度
    constexpr int recover_min_height = 40; // 补给区最小高度
    constexpr int recover_max_height = 60; // 补给区最大高度

    constexpr int password_min_width = 50;  // 密码发射区最小宽度
    constexpr int password_max_width = 70;  // 密码发射区最大宽度
    constexpr int password_min_height = 50; // 密码发射区最小高度
    constexpr int password_max_height = 70; // 密码发射区最大高度

    constexpr int base_min_width = 10;  // 基地最小宽度
    constexpr int base_max_width = 30;  // 基地最大宽度
    constexpr int base_min_height = 10; // 基地最小高度
    constexpr int base_max_height = 30; // 基地最大高度

    constexpr int hp_min_width = 20;  // 生命值条最小宽度
    constexpr int hp_max_width = 400;  // 生命值条最大宽度
    constexpr int hp_min_height = 20; // 生命值条最小高度
    constexpr int hp_max_height = 40; // 生命值条最大高度

    constexpr int our_robot_min_width = 10;  // 己方机器人最小宽度
    constexpr int our_robot_max_width = 50;  // 己方机器人最大宽度
    constexpr int our_robot_min_height = 10; // 己方机器人最小高度
    constexpr int our_robot_max_height = 50; // 己方机器人最大高度

    constexpr int enemy_min_width = 10;  // 敌人最小宽度
    constexpr int enemy_max_width = 50;  // 敌人最大宽度
    constexpr int enemy_min_height = 10; // 敌人最小高度
    constexpr int enemy_max_height = 50; // 敌人最大高度
}

namespace area_color {
    //  密码发射区 rgb(94,101,255)
    const cv::Scalar password_lower(90, 95, 250); // 密码发射区颜色下限
    const cv::Scalar password_upper(100, 105, 255); //密码发射区颜色上限

    // 补给区 rgb(60,85,107)
    const cv::Scalar recover_lower(55, 80, 100); // 补给区颜色下限
    const cv::Scalar recover_upper(65, 90, 115); //补给区颜色上限

    // 基地 rgb(170,120,151)
    const cv::Scalar base_lower(165, 115, 145);// 基地颜色下限
    const cv::Scalar base_upper(175, 125, 155);// 基地颜色上限
    // 地图 rgb(58,58,58)

    const cv::Scalar map_lower(54, 54, 54);// 地图颜色下限
    const cv::Scalar map_upper(62, 62, 62);// 地图颜色上限

    // 紫色出入口 rgb(193,97,212)
    const cv::Scalar purple_lower(190, 95, 210);// 紫色出入口颜色下限
    const cv::Scalar purple_upper(195, 100, 215);// 紫色出入口颜色上限

    // 绿色出入口 rgb(29,198,113)
    const cv::Scalar green_lower(25, 195, 110);// 绿色出入口颜色下限
    const cv::Scalar green_upper(30, 200, 115);// 绿色出入口颜色上限

    // 生命值条 rgb(131,131,131)
    const cv::Scalar hp_block_lower(125, 125, 125);// 生命值条颜色下限
    const cv::Scalar hp_block_upper(137, 137, 137);// 生命值条颜色上限

    // 己方机器人 rgb(89,170,240)
    const cv::Scalar our_robot_lower(85, 165, 235);// 己方机器人颜色下限
    const cv::Scalar our_robot_upper(93, 175, 245);// 己方机器人颜色上限

    // 敌方机器人 rgb(255,104,104)
    const cv::Scalar enemy_lower(250, 100, 100);// 敌方机器人颜色下限
    const cv::Scalar enemy_upper(255, 108, 108);// 敌方机器人颜色上限
}

namespace debug_option {
    constexpr bool print_our_pose = false;
    constexpr bool print_area_info = false;
    constexpr bool print_robot_info = false;
}

#endif // ^^ !_CONSTANT_HPP_