#include "navigation_node.hpp"

navigation::Node::Node(const std::string& name)
    :rclcpp::Node(name)
{
    m_our_pose_publisher = this->create_publisher<geometry_msgs::msg::Pose2D>(topic_name::pose, 1);
    m_shoot_publisher = this->create_publisher<example_interfaces::msg::Bool>(topic_name::shoot, 1);
    m_password_subscription = this->create_subscription<example_interfaces::msg::Int64>(topic_name::password, 1, std::bind(&navigation::Node::password_cbfn, this, std::placeholders::_1));
    m_password_segment_subscription = this->create_subscription<example_interfaces::msg::Int64>(topic_name::password_segment, 1, std::bind(&navigation::Node::password_segment_cbfn, this, std::placeholders::_1));
    m_map_subscription = this->create_subscription<info_interfaces::msg::Map>(topic_name::map, 1, std::bind(&navigation::Node::map_init_cbfn, this, std::placeholders::_1));
    m_area_subscription = this->create_subscription<info_interfaces::msg::Area>(topic_name::area, 1, std::bind(&navigation::Node::area_init_cbfn, this, std::placeholders::_1));
    m_robot_subscription = this->create_subscription<info_interfaces::msg::Robot>(topic_name::robot, 1, std::bind(&navigation::Node::robot_navigation_cbfn, this, std::placeholders::_1));
}

void navigation::Node::map_init_cbfn(const info_interfaces::msg::Map::SharedPtr map_info)
{
    RCLCPP_INFO(get_logger(), "map got!");
    m_map = map_info;
    RCLCPP_INFO(get_logger(), "row:%d col:%d", map_info->row, map_info->col);
    RCLCPP_INFO(get_logger(), "empty:%d", map_info->mat.empty());
}

void navigation::Node::area_init_cbfn(const info_interfaces::msg::Area::SharedPtr area_info)
{
    RCLCPP_INFO(get_logger(), "area got!");
    m_area = *area_info;
}

void navigation::Node::robot_navigation_cbfn(const info_interfaces::msg::Robot::SharedPtr robot_info)
{
    if (!this->m_map.use_count()) return;

    RCLCPP_INFO(get_logger(), "nav");
    RCLCPP_INFO(get_logger(), "src_x:%d src_y:%d dst_x:%d dst_y:%d", robot_info->our_robot.x, robot_info->our_robot.y, robot_info->enemy[0].x, robot_info->enemy[0].y);

    RCLCPP_INFO(get_logger(), "vaild:%ld", this->m_map.use_count());
    RCLCPP_INFO(get_logger(), "row:%d col:%d", this->m_map->row, this->m_map->col);
    RCLCPP_INFO(get_logger(), "empty:%d", this->m_map->mat.empty());

    algorithm::Path path = algorithm::a_star(this->m_map, robot_info->our_robot.x, robot_info->our_robot.y, robot_info->enemy[0].x, robot_info->enemy[0].y, get_logger());
    RCLCPP_INFO(get_logger(), "path is empty %d", path.empty());
    for (auto& point : path) {
        RCLCPP_INFO(get_logger(), "moving");
        geometry_msgs::msg::Pose2D pose;
        std::tie(pose.x, pose.y) = point;
        pose.theta = std::atan2(robot_info->our_robot.y - robot_info->enemy[0].y, robot_info->our_robot.x - robot_info->enemy[0].x);
        m_our_pose_publisher->publish(pose);
    }
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