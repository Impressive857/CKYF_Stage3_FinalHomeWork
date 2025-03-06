#ifndef _NAVIGATION_NODE_HPP_
#define _NAVIGATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/int64.hpp>

#include "info_interfaces/msg/map.hpp"
#include "info_interfaces/msg/area.hpp"
#include "info_interfaces/msg/robot.hpp"

#include "constant.hpp"
#include "my_serial.hpp"
#include "algorithm.hpp"

namespace navigation {
    class Node
        : public rclcpp::Node
    {
    public:
        explicit Node(const std::string& name);
    private:
        enum class Status {
            NONE = 0,
            FIND_ENEMY = 1,
            GET_RECOVER = 2,
            ENTER_GATE = 3,
            SEND_PASSWORD = 4,
            EXIT_GATE = 5,
            FIND_BASE = 6
        };
    private:
        void grid_map_init_cbfn(const info_interfaces::msg::Map::SharedPtr map_info);
        void real_map_init_cbfn(const info_interfaces::msg::Map::SharedPtr map_info);
        void grid_area_init_cbfn(const info_interfaces::msg::Area::SharedPtr area_info);
        void real_area_init_cbfn(const info_interfaces::msg::Area::SharedPtr area_info);
        void robot_navigation_cbfn(const info_interfaces::msg::Robot::SharedPtr robot_info);
        void password_segment_cbfn(const example_interfaces::msg::Int64::SharedPtr password_segment);
        void password_got_cbfn(const my_serial::password_receive_t& password_receive);
        void restart_cbfn(const example_interfaces::msg::Bool::SharedPtr restart_info);
        /// @brief 判断能否攻击
        /// @param src_real src实际地图坐标
        /// @param dst_real dst实际地图坐标
        /// @return 能否攻击
        bool can_attack(const info_interfaces::msg::Point src_real, const info_interfaces::msg::Point dst_real);
        /// @brief 判断是否在附近
        /// @param src_real src实际地图坐标
        /// @param dst_real dst实际地图坐标
        /// @param distance 视为在附近的距离
        /// @return 是否在附近
        bool is_around(const info_interfaces::msg::Point src_real, const info_interfaces::msg::Point dst_real, double distance = constant::near_distance);
        /// @brief 获取朝向角度
        /// @param src src坐标
        /// @param toward 朝向目标坐标
        /// @return 朝向角度
        double get_theta(const info_interfaces::msg::Point src, const info_interfaces::msg::Point toward);
        /// @brief 获取src到dst导航所需pose
        /// @param src_grid src网格地图坐标
        /// @param dst_grid dst网格地图坐标
        /// @return 导航所需pose
        /// @warning 朝向角度是根据网格地图坐标计算出的，精度较低，不适合瞄准，要求瞄准精度清使用指定朝向的重载版本
        geometry_msgs::msg::Pose2D get_pose(const info_interfaces::msg::Point src_grid, const info_interfaces::msg::Point dst_grid);
        /// @brief 获取src到dst导航所需pose，且可以指定朝向
        /// @param src_grid src网格地图坐标
        /// @param dst_grid dst网格地图坐标
        /// @param src_real src实际地图坐标
        /// @param toward_real 朝向目标实际地图坐标
        /// @return 导航所需pose
        geometry_msgs::msg::Pose2D get_pose(const info_interfaces::msg::Point src_grid, const info_interfaces::msg::Point dst_grid, const info_interfaces::msg::Point src_real, const info_interfaces::msg::Point toward_real);
        /// @brief 打印当前状态
        void print_status() const;
    private:
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr m_our_pose_publisher;
        rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr m_shoot_publisher;
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr m_password_publisher;
        rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr m_password_segment_subscription;
        rclcpp::Subscription<info_interfaces::msg::Map>::SharedPtr m_grid_map_subscription;
        rclcpp::Subscription<info_interfaces::msg::Map>::SharedPtr m_real_map_subscription;
        rclcpp::Subscription<info_interfaces::msg::Area>::SharedPtr m_grid_area_subscription;
        rclcpp::Subscription<info_interfaces::msg::Area>::SharedPtr m_real_area_subscription;
        rclcpp::Subscription<info_interfaces::msg::Robot>::SharedPtr m_robot_subscription;
        rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr m_restart_subscription;
        info_interfaces::msg::Area::SharedPtr m_grid_area;
        info_interfaces::msg::Area::SharedPtr m_real_area;
        info_interfaces::msg::Map::SharedPtr m_grid_map;
        info_interfaces::msg::Map::SharedPtr m_real_map;
        example_interfaces::msg::Int64 m_password;
        std::vector<example_interfaces::msg::Int64> m_password_segment_vec;
        int m_dir_count; // 防止卡死时，记录方向的计数器
        int m_count; // 防止卡死时的计数器
        int m_bullet_num;
        bool m_should_stop;
        bool m_need_recover;
        bool m_password_segment_has_sent;
        bool m_password_has_got;
        bool m_password_has_sent;
        bool m_grid_map_has_got;
        bool m_real_map_has_got;
        bool m_grid_area_has_got;
        bool m_real_area_has_got;
        double m_last_hp;
        uint32_t m_last_real_x;
        uint32_t m_last_real_y;
        my_serial::MySerial m_serial;
        my_serial::password_receive_t m_password_receive;
        Status m_current_status;
        Status m_last_status;
        const int m_dir[4][2]{ {-1,-1},{1,-1},{1,1},{-1,1} };
    };
}

#endif // ^^ !_NAVIGATION_NODE_HPP_