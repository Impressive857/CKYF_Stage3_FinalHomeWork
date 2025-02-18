#include "navigation_node.hpp"

navigation::Node::Node(const std::string& name)
    :rclcpp::Node(name)
{
    m_count = 0;
    m_last_x = 0;
    m_last_y = 0;
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
}

void navigation::Node::area_init_cbfn(const info_interfaces::msg::Area::SharedPtr area_info)
{
    RCLCPP_INFO(get_logger(), "area got!");
    m_area = *area_info;
}

void navigation::Node::robot_navigation_cbfn(const info_interfaces::msg::Robot::SharedPtr robot_info)
{
    if (!this->m_map.use_count()) return;

    geometry_msgs::msg::Pose2D pose;

    if (0 == m_last_x && 0 == m_last_y) {
        m_last_x = robot_info->our_robot.x;
        m_last_y = robot_info->our_robot.y;
    }
    // 防止卡死在一个位置
    else if (robot_info->our_robot.x == m_last_x && robot_info->our_robot.y == m_last_y) {
        m_last_x = robot_info->our_robot.x;
        m_last_y = robot_info->our_robot.y;
        pose.x = m_dir[m_count][0];
        pose.y = m_dir[m_count][1];
        pose.theta = 0;
        m_count++;
        m_count %= 4;
        m_our_pose_publisher->publish(pose);
    }
    else {
        if (robot_info->enemy.empty()) return;
        std::sort(robot_info->enemy.begin(), robot_info->enemy.end(), [&robot_info](const info_interfaces::msg::Point& a, const info_interfaces::msg::Point& b)
            {return algorithm::manhattan_distance(robot_info->our_robot.x, robot_info->our_robot.y, a.x, a.y) < algorithm::manhattan_distance(robot_info->our_robot.x, robot_info->our_robot.y, b.x, b.y);});
        algorithm::Path path = algorithm::a_star(
            this->m_map,
            robot_info->our_robot.x,
            robot_info->our_robot.y,
            robot_info->enemy[0].x,
            robot_info->enemy[0].y
        );
        RCLCPP_INFO(get_logger(), "ourx:%d, oury:%d", robot_info->our_robot.x, robot_info->our_robot.y);
        RCLCPP_INFO(get_logger(), "enemyx:%d, enemyy:%d", robot_info->enemy[0].x, robot_info->enemy[0].y);

        if (path.size() >= 2) {
            std::tie(pose.x, pose.y) = path[1];
            RCLCPP_INFO(get_logger(), "pathx:%d, pathy:%d", path[1].first, path[1].second);
            pose.x -= robot_info->our_robot.x;
            pose.y -= robot_info->our_robot.y;
            pose.x /= 16;
            pose.y /= 16;
            pose.theta = std::atan2(robot_info->enemy[0].y - robot_info->our_robot.y, robot_info->enemy[0].x - robot_info->our_robot.x);
            RCLCPP_INFO(get_logger(), "posex:%lf, posey:%lf theta:%lf", pose.x, pose.y, pose.theta);
            m_our_pose_publisher->publish(pose);
        }
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