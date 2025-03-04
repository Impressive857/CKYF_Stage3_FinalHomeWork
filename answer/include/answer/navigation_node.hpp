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
        bool can_attack(const info_interfaces::msg::Map::SharedPtr map_info, int src_x, int src_y, int dst_x, int dst_y);
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
        int m_count;
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