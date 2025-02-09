#include "navigation_node.hpp"

navigation::Node::Node(const std::string& name)
    :rclcpp::Node(name)
{
    m_our_pose_publisher = this->create_publisher<geometry_msgs::msg::Pose>(topic_name::pose, 1);
    m_shoot_publisher = this->create_publisher<example_interfaces::msg::Bool>(topic_name::shoot, 1);
    m_password_subscription = this->create_subscription<example_interfaces::msg::Int64>(topic_name::password, 1, std::bind(&navigation::Node::password_cbfn, this, std::placeholders::_1));
    m_password_segment_subscription = this->create_subscription<example_interfaces::msg::Int64>(topic_name::password_segment, 1, std::bind(&navigation::Node::password_segment_cbfn, this, std::placeholders::_1));
    m_map_subscription = this->create_subscription<info_interfaces::msg::Map>(topic_name::map, 5, std::bind(&navigation::Node::map_init_cbfn, this, std::placeholders::_1));
    m_area_subscription = this->create_subscription<info_interfaces::msg::Area>(topic_name::area, 1, std::bind(&navigation::Node::area_init_cbfn, this, std::placeholders::_1));
    m_robot_subscription = this->create_subscription<info_interfaces::msg::Robot>(topic_name::robot, 1, std::bind(&navigation::Node::robot_navigation_cbfn, this, std::placeholders::_1));
}

void navigation::Node::map_init_cbfn(const info_interfaces::msg::Map::SharedPtr map_info)
{
    m_map = *map_info;
}

void navigation::Node::area_init_cbfn(const info_interfaces::msg::Area::SharedPtr area_info)
{
    m_area = *area_info;
}

void navigation::Node::robot_navigation_cbfn(const info_interfaces::msg::Robot::SharedPtr robot_info)
{
}

void navigation::Node::password_cbfn(const example_interfaces::msg::Int64::SharedPtr password)
{
    m_password = *password;
}

void navigation::Node::password_segment_cbfn(const example_interfaces::msg::Int64::SharedPtr password_segment)
{
    m_password_segment_vec.push_back(*password_segment);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<navigation::Node>(std::string("navigation_node"));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}