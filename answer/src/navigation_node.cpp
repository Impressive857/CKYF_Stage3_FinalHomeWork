#include "navigation_node.hpp"

navigation::Node::Node(const std::string& name)
    :rclcpp::Node(name), m_serial(constant::serial_path, constant::baud_rate)
{
    m_count = 0;
    m_dir_count = 0;
    m_last_real_x = 0;
    m_last_real_y = 0;
    m_bullet_num = 10;
    m_should_stop = false;
    m_need_recover = false;
    m_password_has_got = false;
    m_password_segment_has_sent = false;
    m_password_has_sent = false;
    m_grid_map_has_got = false;
    m_real_map_has_got = false;
    m_grid_area_has_got = false;
    m_real_area_has_got = false;
    m_current_status = Status::NONE;
    m_last_status = Status::NONE;
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
    // 确保map有效
    if (!this->m_grid_map.use_count()) return;
    if (!this->m_real_map.use_count()) return;

    // 按网格距离对敌人排序
    std::sort(robot_info->enemy_grid_pos_vec.begin(), robot_info->enemy_grid_pos_vec.end(), [&robot_info](const info_interfaces::msg::Point& a, const info_interfaces::msg::Point& b)
        {return algorithm::manhattan_distance(robot_info->our_robot_grid_pos.x, robot_info->our_robot_grid_pos.y, a.x, a.y) < algorithm::manhattan_distance(robot_info->our_robot_grid_pos.x, robot_info->our_robot_grid_pos.y, b.x, b.y);});
    // 按实际距离对敌人排序
    std::sort(robot_info->enemy_real_pos_vec.begin(), robot_info->enemy_real_pos_vec.end(), [&robot_info](const info_interfaces::msg::Point& a, const info_interfaces::msg::Point& b)
        {return algorithm::manhattan_distance(robot_info->our_robot_real_pos.x, robot_info->our_robot_real_pos.y, a.x, a.y) < algorithm::manhattan_distance(robot_info->our_robot_real_pos.x, robot_info->our_robot_real_pos.y, b.x, b.y);});

    geometry_msgs::msg::Pose2D pose;
    example_interfaces::msg::Bool shoot;
    shoot.data = true;
    algorithm::Path path;
    m_should_stop = false;

    // 防止卡死在一个位置
    // 为了防止短时间内的快速循环导致误判卡死
    // 到达一定的循环次数才进行脱离卡死状态操作
    if (robot_info->our_robot_real_pos.x == m_last_real_x && robot_info->our_robot_real_pos.y == m_last_real_y && !m_should_stop && ++m_count > 10) {
        m_count = 0;

        pose.x = m_dir[m_dir_count][0];
        pose.y = m_dir[m_dir_count][1];
        pose.x *= constant::speed_scale;
        pose.y *= constant::speed_scale;
        // 如果有敌人，防卡死时确保指向敌人
        if (!robot_info->enemy_real_pos_vec.empty()) {
            pose.theta = get_theta(robot_info->our_robot_real_pos, robot_info->enemy_real_pos_vec[0]);
        }
        // 如果没有敌人，防卡死时指向基地
        else {
            pose.theta = get_theta(robot_info->our_robot_real_pos, m_real_area->base_pos);
        }
        m_count++;
        m_count %= 4;
    }
    // 如果机器人状态不佳且有敌方机器人存在，则前去补给
    else if ((robot_info->our_robot_hp < constant::danger_hp || m_need_recover || m_bullet_num < constant::danger_bullet_num) && m_password_segment_vec.size() < 2) {
        m_current_status = Status::GET_RECOVER;

        m_need_recover = robot_info->our_robot_hp >= constant::safe_hp;

        if (!robot_info->enemy_grid_pos_vec.empty()) {
            pose = get_pose(robot_info->our_robot_grid_pos, m_grid_area->recover_pos, robot_info->our_robot_real_pos, robot_info->enemy_real_pos_vec[0]);

            if (can_attack(robot_info->our_robot_real_pos, robot_info->enemy_real_pos_vec[0]))
            {
                m_shoot_publisher->publish(shoot);
            }
        }
        else {
            pose = get_pose(robot_info->our_robot_grid_pos, m_grid_area->recover_pos);
        }

    }
    // 如果两段密码片段已经接收且密码片段未发送，则发送密码片段
    else if (m_password_segment_vec.size() >= 2 && !m_password_segment_has_sent) {
        my_serial::password_send_t password_send;
        password_send.password1 = m_password_segment_vec[0].data;
        password_send.password2 = m_password_segment_vec[1].data;
        m_serial.write(password_send);
        m_password_segment_has_sent = true;
        RCLCPP_INFO(get_logger(), "password segment has sent!");
    }
    // 如果密码片段已经发送且密码已经接收到，则准备进攻基地
    else if (m_password_segment_has_sent && m_password_has_got) {
        // 机器人和密码发射区无法直线到达，说明未进入密码发射区，前往入口
        if (!algorithm::can_connect(m_real_map, robot_info->our_robot_real_pos.x, robot_info->our_robot_real_pos.y, m_real_area->password_pos.x, m_real_area->password_pos.y) && !m_password_has_sent) {
            m_current_status = Status::ENTER_GATE;

            m_should_stop = is_around(robot_info->our_robot_real_pos, m_real_area->enter_gate_pos);

            pose = get_pose(robot_info->our_robot_grid_pos, m_grid_area->enter_gate_pos);
        }
        // 进入密码发射区，未发送密码则发送密码
        else if (!m_password_has_sent) {
            m_current_status = Status::SEND_PASSWORD;

            m_should_stop = is_around(robot_info->our_robot_real_pos, m_real_area->password_pos, constant::send_password_distance);

            pose = get_pose(robot_info->our_robot_grid_pos, m_grid_area->password_pos);

            if (m_should_stop) {
                m_password_publisher->publish(m_password);
                RCLCPP_INFO(get_logger(), "password has send!");
                m_password_has_sent = true;
            }
        }
        // 密码已经发送且机器人能和出口直线到达，说明未离开密码发射区，前往出口
        else if (m_password_has_sent && algorithm::can_connect(m_real_map, robot_info->our_robot_real_pos.x, robot_info->our_robot_real_pos.y, m_real_area->exit_gate_pos.x, m_real_area->exit_gate_pos.y)) {
            m_current_status = Status::EXIT_GATE;

            m_should_stop = is_around(robot_info->our_robot_real_pos, m_real_area->exit_gate_pos);

            pose = get_pose(robot_info->our_robot_grid_pos, m_grid_area->exit_gate_pos);
        }
        // 密码已经发送且机器人离开密码发射区，则前往基地
        else {
            m_current_status = Status::FIND_BASE;

            m_should_stop = is_around(robot_info->our_robot_real_pos, m_real_area->base_pos, constant::attack_base_distance);

            pose = get_pose(robot_info->our_robot_grid_pos, m_grid_area->base_pos, robot_info->our_robot_real_pos, m_real_area->base_pos);

            if (can_attack(robot_info->our_robot_real_pos, m_real_area->base_pos))
            {
                m_shoot_publisher->publish(shoot);
            }
        }
    }
    // 如果还有敌人，则攻击敌人
    else if (!robot_info->enemy_grid_pos_vec.empty())
    {
        m_current_status = Status::FIND_ENEMY;

        pose = get_pose(robot_info->our_robot_grid_pos, robot_info->enemy_grid_pos_vec[0], robot_info->our_robot_real_pos, robot_info->enemy_real_pos_vec[0]);

        if (can_attack(robot_info->our_robot_real_pos, robot_info->enemy_real_pos_vec[0]))
        {
            m_shoot_publisher->publish(shoot);
        }
    }

    m_our_pose_publisher->publish(pose);

    // 如果状态改变则打印
    if (m_current_status != m_last_status) {
        print_status();
    }

    // 更新状态
    m_last_status = m_current_status;

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
        m_dir_count = 0;
        m_bullet_num = 10;
        m_should_stop = false;
        m_need_recover = false;
        m_password_segment_has_sent = false;
        m_password_has_got = false;
        m_password_has_sent = false;
        m_last_real_x = 0;
        m_last_real_y = 0;
        m_real_area_has_got = false;
        m_grid_area_has_got = false;
        m_real_map_has_got = false;
        m_grid_map_has_got = false;
        m_current_status = Status::NONE;
        m_last_status = Status::NONE;
    }
}

bool navigation::Node::can_attack(const info_interfaces::msg::Point src_real, const info_interfaces::msg::Point dst_real)
{
    return (
        algorithm::can_connect(m_real_map, src_real.x, src_real.y, dst_real.x, dst_real.y)
        &&
        algorithm::euclidean_distance(src_real.x, src_real.y, dst_real.x, dst_real.y) < static_cast<double>(constant::attack_distance)
        &&
        m_bullet_num > 0
        );
}

bool navigation::Node::is_around(const info_interfaces::msg::Point src_real, const info_interfaces::msg::Point dst_real, double distance)
{
    return algorithm::euclidean_distance(src_real.x, src_real.y, dst_real.x, dst_real.y) < distance;
}

double navigation::Node::get_theta(const info_interfaces::msg::Point src, const info_interfaces::msg::Point toward)
{
    int32_t dy = static_cast<int32_t>(toward.y) - static_cast<int32_t>(src.y);
    int32_t dx = static_cast<int32_t>(toward.x) - static_cast<int32_t>(src.x);
    return std::atan2(dy, dx);
}

geometry_msgs::msg::Pose2D navigation::Node::get_pose(const info_interfaces::msg::Point src_grid, const info_interfaces::msg::Point dst_grid) {
    geometry_msgs::msg::Pose2D pose;
    algorithm::Path path;

    pose.theta = get_theta(src_grid, dst_grid);

    if (!m_should_stop) {
        path = algorithm::a_star(m_grid_map, src_grid.x, src_grid.y, dst_grid.x, dst_grid.y);

        if (path.size() >= 2) {
            std::tie(pose.x, pose.y) = path[1];
            pose.x -= src_grid.x;
            pose.y -= src_grid.y;
            pose.x *= constant::speed_scale;
            pose.y *= constant::speed_scale;
        }
    }
    else {
        pose.x = 0;
        pose.y = 0;
    }

    if constexpr (debug_option::print_our_pose) {
        RCLCPP_INFO(get_logger(), "posex:%lf, posey:%lf theta:%lf", pose.x, pose.y, pose.theta);
    }

    return pose;
}

geometry_msgs::msg::Pose2D navigation::Node::get_pose(const info_interfaces::msg::Point src_grid, const info_interfaces::msg::Point dst_grid, const info_interfaces::msg::Point src_real, const info_interfaces::msg::Point toward_real) {
    geometry_msgs::msg::Pose2D pose;
    algorithm::Path path;

    pose.theta = get_theta(src_real, toward_real);

    if (!m_should_stop) {
        path = algorithm::a_star(m_grid_map, src_grid.x, src_grid.y, dst_grid.x, dst_grid.y);

        if (path.size() >= 2) {
            std::tie(pose.x, pose.y) = path[1];
            pose.x -= src_grid.x;
            pose.y -= src_grid.y;
            pose.x *= constant::speed_scale;
            pose.y *= constant::speed_scale;
        }
    }
    else {
        pose.x = 0;
        pose.y = 0;
    }

    if constexpr (debug_option::print_our_pose) {
        RCLCPP_INFO(get_logger(), "posex:%lf, posey:%lf theta:%lf", pose.x, pose.y, pose.theta);
    }

    return pose;
}

void navigation::Node::print_status() const
{
    switch (m_current_status) {
    case Status::NONE: {
        break;
    }
    case Status::FIND_ENEMY: {
        RCLCPP_INFO(get_logger(), "finding enemy!");
        break;
    }
    case Status::GET_RECOVER: {
        RCLCPP_INFO(get_logger(), "getting recover!");
        break;
    }
    case Status::ENTER_GATE: {
        RCLCPP_INFO(get_logger(), "going to enter gate!");
        break;
    }
    case Status::SEND_PASSWORD: {
        RCLCPP_INFO(get_logger(), "sending password!");
        break;
    }
    case Status::EXIT_GATE: {
        RCLCPP_INFO(get_logger(), "going to exit gate!");
        break;
    }
    case Status::FIND_BASE: {
        RCLCPP_INFO(get_logger(), "finding base!");
        break;
    }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<navigation::Node>(std::string("navigation_node"));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}