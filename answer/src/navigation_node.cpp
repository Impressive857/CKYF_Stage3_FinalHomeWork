#include "navigation_node.hpp"

navigation::Node::Node(const std::string& name)
    :rclcpp::Node(name), m_serial(constant::serial_path, constant::baud_rate)
{
    m_count = 0;
    m_last_real_x = 0;
    m_last_real_y = 0;
    m_bullet_num = 10;
    m_last_hp = 1.0;
    m_should_stop = false;
    m_need_recover = false;
    m_password_has_got = false;
    m_password_segment_has_sent = false;
    m_password_has_sent = false;
    m_grid_map_has_got = false;
    m_real_map_has_got = false;
    m_grid_area_has_got = false;
    m_real_area_has_got = false;
    m_serial.spin(true);
    m_serial.registerCallback(my_serial::CMD_READ, std::function<void(const my_serial::password_receive_t&)>(std::bind(&navigation::Node::password_got_cbfn, this, std::placeholders::_1)));
    m_our_pose_publisher = this->create_publisher<geometry_msgs::msg::Pose2D>(topic_name::pose, 1);
    m_shoot_publisher = this->create_publisher<example_interfaces::msg::Bool>(topic_name::shoot, 1);
    m_password_publisher = this->create_publisher<example_interfaces::msg::Int64>(topic_name::password, 1);
    m_password_segment_subscription = this->create_subscription<example_interfaces::msg::Int64>(topic_name::password_segment, 1, std::bind(&navigation::Node::password_segment_cbfn, this, std::placeholders::_1));
    m_grid_map_subscription = this->create_subscription<info_interfaces::msg::Map>(topic_name::grid_map, 1, std::bind(&navigation::Node::grid_map_init_cbfn, this, std::placeholders::_1));
    m_real_map_subscription = this->create_subscription<info_interfaces::msg::Map>(topic_name::real_map, 1, std::bind(&navigation::Node::real_map_init_cbfn, this, std::placeholders::_1));
    m_grid_area_subscription = this->create_subscription<info_interfaces::msg::Area>(topic_name::grid_area, 1, std::bind(&navigation::Node::grid_area_init_cbfn, this, std::placeholders::_1));
    m_real_area_subscription = this->create_subscription<info_interfaces::msg::Area>(topic_name::real_area, 1, std::bind(&navigation::Node::real_area_init_cbfn, this, std::placeholders::_1));
    m_robot_subscription = this->create_subscription<info_interfaces::msg::Robot>(topic_name::robot, 1, std::bind(&navigation::Node::robot_navigation_cbfn, this, std::placeholders::_1));
    m_restart_subscription = this->create_subscription<example_interfaces::msg::Bool>(topic_name::restart, 1, std::bind(&navigation::Node::restart_cbfn, this, std::placeholders::_1));
}

void navigation::Node::grid_map_init_cbfn(const info_interfaces::msg::Map::SharedPtr map_info)
{
    RCLCPP_INFO(get_logger(), "grid_map got!");
    m_grid_map = map_info;
    m_grid_map_has_got = true;
}

void navigation::Node::real_map_init_cbfn(const info_interfaces::msg::Map::SharedPtr map_info)
{
    RCLCPP_INFO(get_logger(), "real_map got!");
    m_real_map = map_info;
    m_real_map_has_got = true;
}

void navigation::Node::grid_area_init_cbfn(const info_interfaces::msg::Area::SharedPtr area_info)
{
    RCLCPP_INFO(get_logger(), "grid area got!");
    m_grid_area_has_got = true;
    m_grid_area = area_info;
}

void navigation::Node::real_area_init_cbfn(const info_interfaces::msg::Area::SharedPtr area_info)
{
    RCLCPP_INFO(get_logger(), "real area got!");
    m_real_area = area_info;
    m_real_area_has_got = true;
}

void navigation::Node::robot_navigation_cbfn(const info_interfaces::msg::Robot::SharedPtr robot_info)
{
    if (!this->m_grid_map.use_count()) return;
    if (!this->m_real_map.use_count()) return;

    // 按网格距离对敌人排序
    std::sort(robot_info->enemy_grid_pos_vec.begin(), robot_info->enemy_grid_pos_vec.end(), [&robot_info](const info_interfaces::msg::Point& a, const info_interfaces::msg::Point& b)
        {return algorithm::manhattan_distance(robot_info->our_robot_grid_pos.x, robot_info->our_robot_grid_pos.y, a.x, a.y) < algorithm::manhattan_distance(robot_info->our_robot_grid_pos.x, robot_info->our_robot_grid_pos.y, b.x, b.y);});
    // 按实际距离对敌人排序
    std::sort(robot_info->enemy_real_pos_vec.begin(), robot_info->enemy_real_pos_vec.end(), [&robot_info](const info_interfaces::msg::Point& a, const info_interfaces::msg::Point& b)
        {return algorithm::manhattan_distance(robot_info->our_robot_real_pos.x, robot_info->our_robot_real_pos.y, a.x, a.y) < algorithm::manhattan_distance(robot_info->our_robot_real_pos.x, robot_info->our_robot_real_pos.y, b.x, b.y);});

    geometry_msgs::msg::Pose2D pose;

    // 防止卡死在一个位置
    if (robot_info->our_robot_real_pos.x == m_last_real_x && robot_info->our_robot_real_pos.y == m_last_real_y && !m_should_stop) {
        pose.x = m_dir[m_count][0];
        pose.y = m_dir[m_count][1];
        if (robot_info->enemy_real_pos_vec.empty()) {
            pose.theta = 0;
        }
        else {
            int32_t dy = static_cast<int32_t>(robot_info->enemy_real_pos_vec[0].y) - static_cast<int32_t>(robot_info->our_robot_real_pos.y);
            int32_t dx = static_cast<int32_t>(robot_info->enemy_real_pos_vec[0].x) - static_cast<int32_t>(robot_info->our_robot_real_pos.x);
            pose.theta = std::atan2(dy, dx);
        }
        m_count++;
        m_count %= 4;
        m_our_pose_publisher->publish(pose);
    }
    else if ((robot_info->our_robot_hp < constant::danger_hp || m_need_recover || m_bullet_num < constant::danger_bullet_num) && m_password_segment_vec.size() < 2) {
        m_need_recover = true;
        if (robot_info->our_robot_hp >= 1.0) {
            m_need_recover = false;
            return;
        }
        algorithm::Path path = algorithm::a_star(
            this->m_grid_map,
            robot_info->our_robot_grid_pos.x,
            robot_info->our_robot_grid_pos.y,
            m_grid_area->recover_pos.x,
            m_grid_area->recover_pos.y
        );

        if (path.size() >= 2) {
            // RCLCPP_INFO(get_logger(), "going to recover area!");
            std::tie(pose.x, pose.y) = path[1];
            pose.x -= robot_info->our_robot_grid_pos.x;
            pose.y -= robot_info->our_robot_grid_pos.y;
            pose.x *= constant::speed_scale;
            pose.y *= constant::speed_scale;
            if (!robot_info->enemy_grid_pos_vec.empty()) {
                int32_t dy = static_cast<int32_t>(robot_info->enemy_real_pos_vec[0].y) - static_cast<int32_t>(robot_info->our_robot_real_pos.y);
                int32_t dx = static_cast<int32_t>(robot_info->enemy_real_pos_vec[0].x) - static_cast<int32_t>(robot_info->our_robot_real_pos.x);
                pose.theta = std::atan2(dy, dx);

                example_interfaces::msg::Bool shoot;
                if (algorithm::can_connect(m_real_map, robot_info->our_robot_real_pos.x, robot_info->our_robot_real_pos.y, robot_info->enemy_real_pos_vec[0].x, robot_info->enemy_real_pos_vec[0].y)) {
                    shoot.data = true;
                    m_shoot_publisher->publish(shoot);
                }
            }
            else {
                pose.theta = 0;
            }
            m_our_pose_publisher->publish(pose);
        }

        // if (robot_info->our_robot_hp > m_last_hp) {
        //     m_bullet_num += 10;
        // }
        m_last_hp = robot_info->our_robot_hp;
    }
    else if (m_password_segment_vec.size() >= 2 && !m_password_segment_has_sent) {
        my_serial::password_send_t password_send;
        password_send.password1 = m_password_segment_vec[0].data;
        password_send.password2 = m_password_segment_vec[1].data;
        m_serial.write(password_send);
        m_password_segment_has_sent = true;
        RCLCPP_INFO(get_logger(), "password segment has sent!");
    }
    else if (m_password_segment_has_sent && m_password_has_got) {
        algorithm::Path path;
        // 机器人和密码发射区无法直线到达，说明未进入密码发射区
        if (!algorithm::can_connect(m_real_map, robot_info->our_robot_real_pos.x, robot_info->our_robot_real_pos.y, m_real_area->password_pos.x, m_real_area->password_pos.y) && !m_password_has_sent) {
            if (algorithm::manhattan_distance(robot_info->our_robot_real_pos.x, robot_info->our_robot_real_pos.y, m_real_area->enter_gate_pos.x, m_real_area->enter_gate_pos.y) < constant::stop_distance) {
                m_should_stop = true;
                pose.x = 0;
                pose.y = 0;
                pose.theta = 0;
                m_our_pose_publisher->publish(pose);
                return;
            }
            else {
                m_should_stop = false;
                path = algorithm::a_star(
                    this->m_grid_map,
                    robot_info->our_robot_grid_pos.x,
                    robot_info->our_robot_grid_pos.y,
                    m_grid_area->enter_gate_pos.x,
                    m_grid_area->enter_gate_pos.y
                );
            }
        }
        else if (!m_password_has_sent) {
            path = algorithm::a_star(
                this->m_grid_map,
                robot_info->our_robot_grid_pos.x,
                robot_info->our_robot_grid_pos.y,
                m_grid_area->password_pos.x,
                m_grid_area->password_pos.y
            );
            if (algorithm::manhattan_distance(robot_info->our_robot_real_pos.x, robot_info->our_robot_real_pos.y, m_real_area->password_pos.x, m_real_area->password_pos.y) < constant::stop_distance) {
                m_password_publisher->publish(m_password);
                RCLCPP_INFO(get_logger(), "password has send!");
                m_password_has_sent = true;
            }
        }
        else if (m_password_has_sent && algorithm::can_connect(m_real_map, robot_info->our_robot_real_pos.x, robot_info->our_robot_real_pos.y, m_real_area->exit_gate_pos.x, m_real_area->exit_gate_pos.y)) {
            if (algorithm::manhattan_distance(robot_info->our_robot_real_pos.x, robot_info->our_robot_real_pos.y, m_real_area->exit_gate_pos.x, m_real_area->exit_gate_pos.y) < constant::stop_distance) {
                m_should_stop = true;
                pose.x = 0;
                pose.y = 0;
                pose.theta = 0;
                m_our_pose_publisher->publish(pose);
                return;
            }
            else {
                m_should_stop = false;
                path = algorithm::a_star(
                    this->m_grid_map,
                    robot_info->our_robot_grid_pos.x,
                    robot_info->our_robot_grid_pos.y,
                    m_grid_area->exit_gate_pos.y,
                    m_grid_area->exit_gate_pos.y
                );
            }
        }
        else {
            path = algorithm::a_star(
                this->m_grid_map,
                robot_info->our_robot_grid_pos.x,
                robot_info->our_robot_grid_pos.y,
                m_grid_area->base_pos.x,
                m_grid_area->base_pos.y
            );
            example_interfaces::msg::Bool shoot;
            if (algorithm::can_connect(m_real_map, robot_info->our_robot_real_pos.x, robot_info->our_robot_real_pos.y, m_real_area->base_pos.x, m_real_area->base_pos.y)) {
                shoot.data = true;
                m_shoot_publisher->publish(shoot);
            }
            int32_t dy = static_cast<int32_t>(m_real_area->base_pos.y) - static_cast<int32_t>(robot_info->our_robot_real_pos.y);
            int32_t dx = static_cast<int32_t>(m_real_area->base_pos.x) - static_cast<int32_t>(robot_info->our_robot_real_pos.x);
            pose.theta = std::atan2(dy, dx);
        }
        if (path.size() >= 2) {
            std::tie(pose.x, pose.y) = path[1];
            pose.x -= robot_info->our_robot_grid_pos.x;
            pose.y -= robot_info->our_robot_grid_pos.y;
            pose.x *= constant::speed_scale;
            pose.y *= constant::speed_scale;
            if constexpr (debug_option::print_our_pose) {
                RCLCPP_INFO(get_logger(), "posex:%lf, posey:%lf theta:%lf", pose.x, pose.y, pose.theta);
            }
            m_our_pose_publisher->publish(pose);
        }
    }
    else if (!robot_info->enemy_grid_pos_vec.empty())
    {
        algorithm::Path path = algorithm::a_star(
            this->m_grid_map,
            robot_info->our_robot_grid_pos.x,
            robot_info->our_robot_grid_pos.y,
            robot_info->enemy_grid_pos_vec[0].x,
            robot_info->enemy_grid_pos_vec[0].y
        );

        if (path.size() >= 2) {
            std::tie(pose.x, pose.y) = path[1];
            pose.x -= robot_info->our_robot_grid_pos.x;
            pose.y -= robot_info->our_robot_grid_pos.y;
            pose.x *= constant::speed_scale;
            pose.y *= constant::speed_scale;
            int32_t dy = static_cast<int32_t>(robot_info->enemy_real_pos_vec[0].y) - static_cast<int32_t>(robot_info->our_robot_real_pos.y);
            int32_t dx = static_cast<int32_t>(robot_info->enemy_real_pos_vec[0].x) - static_cast<int32_t>(robot_info->our_robot_real_pos.x);
            pose.theta = std::atan2(dy, dx);
            if constexpr (debug_option::print_our_pose) {
                RCLCPP_INFO(get_logger(), "posex:%lf, posey:%lf theta:%lf", pose.x, pose.y, pose.theta);
            }
            m_our_pose_publisher->publish(pose);

            example_interfaces::msg::Bool shoot;
            if (algorithm::can_connect(m_real_map, robot_info->our_robot_real_pos.x, robot_info->our_robot_real_pos.y, robot_info->enemy_real_pos_vec[0].x, robot_info->enemy_real_pos_vec[0].y)) {
                shoot.data = true;
                m_shoot_publisher->publish(shoot);
            }
        }
    }
    // 更新位置
    m_last_real_x = robot_info->our_robot_real_pos.x;
    m_last_real_y = robot_info->our_robot_real_pos.y;
}

void navigation::Node::password_segment_cbfn(const example_interfaces::msg::Int64::SharedPtr password_segment)
{
    m_password_segment_vec.push_back(*password_segment);
    RCLCPP_INFO(get_logger(), "password segment got:%ld", password_segment->data);
}

void navigation::Node::password_got_cbfn(const my_serial::password_receive_t& password_receive)
{
    m_password_receive = password_receive;
    m_password_has_got = true;
    RCLCPP_INFO(get_logger(), "password has got:%ld", m_password_receive.password);
}

void navigation::Node::restart_cbfn(const example_interfaces::msg::Bool::SharedPtr restart_info)
{
    if (true == restart_info->data) {
        m_password_segment_vec.clear();
        m_count = 0;
        m_bullet_num = 10;
        m_should_stop = false;
        m_need_recover = false;
        m_password_segment_has_sent = false;
        m_password_has_got = false;
        m_password_has_sent = false;
        m_last_hp = 1.0;
        m_last_real_x = 0;
        m_last_real_y = 0;
        m_real_area_has_got = false;
        m_grid_area_has_got = false;
        m_real_map_has_got = false;
        m_grid_map_has_got = false;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<navigation::Node>(std::string("navigation_node"));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}