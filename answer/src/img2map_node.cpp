#include "img2map_node.hpp"

img2map::Node::Node(const std::string& name)
    :rclcpp::Node(name)
{
    m_initialized = false;
    m_hp_block_width = 0;
    m_restart_info.data = true;
    m_grid_map_publisher = this->create_publisher<info_interfaces::msg::Map>(topic_name::grid_map, 1);
    m_real_map_publisher = this->create_publisher<info_interfaces::msg::Map>(topic_name::real_map, 1);
    m_grid_area_publisher = this->create_publisher<info_interfaces::msg::Area>(topic_name::grid_area, 1);
    m_real_area_publisher = this->create_publisher<info_interfaces::msg::Area>(topic_name::real_area, 1);
    m_robot_publisher = this->create_publisher<info_interfaces::msg::Robot>(topic_name::robot, 1);
    m_restart_publisher = this->create_publisher<example_interfaces::msg::Bool>(topic_name::restart, 1);
    m_img_subscription = this->create_subscription<sensor_msgs::msg::Image>(topic_name::image_raw, 10, std::bind(&img2map::Node::img2map_cbfn, this, std::placeholders::_1));
}

void img2map::Node::img2map_cbfn(const sensor_msgs::msg::Image::SharedPtr ros_raw_img)
{
    cv_bridge::CvImagePtr cv_raw_img = cv_bridge::toCvCopy(ros_raw_img, ros_raw_img->encoding);
    std::vector<cv::Point> center_pos_vec; // 存储区域中心坐标
    std::vector<cv::Rect> border_rect_vec; // 存储区域边界矩形
    const int grid_width = cv_raw_img->image.cols / constant::grid_num_v; // 网格宽度
    const int grid_height = cv_raw_img->image.rows / constant::grid_num_h; // 网格高度
    if (!m_initialized) {
        RCLCPP_INFO(get_logger(), "map info init");

        // 开始获取area
        info_interfaces::msg::Area grid_area;
        info_interfaces::msg::Area real_area;

        // 获取密码发射区
        std::tie(center_pos_vec, border_rect_vec) = get_area("password", cv_raw_img->image, area_color::password_lower, area_color::password_upper, constant::password_min_width, constant::password_min_height, constant::password_max_width, constant::password_max_height);
        if (center_pos_vec.empty() || border_rect_vec.empty()) {
            RCLCPP_INFO(get_logger(), "failed to get password area!");
            publish_restart_info();
            return;
        }
        // 获取实际坐标
        real_area.password_pos.x = center_pos_vec[0].x;
        real_area.password_pos.y = center_pos_vec[0].y;

        // 获取网格坐标
        grid_area.password_pos.x = center_pos_vec[0].x / grid_width;
        grid_area.password_pos.y = center_pos_vec[0].y / grid_height;

        // 获取补给区
        std::tie(center_pos_vec, border_rect_vec) = get_area("recover", cv_raw_img->image, area_color::recover_lower, area_color::recover_upper, constant::recover_min_width, constant::recover_min_height, constant::recover_max_width, constant::recover_max_height);
        if (center_pos_vec.empty() || border_rect_vec.empty()) {
            RCLCPP_INFO(get_logger(), "failed to get recover area!");
            publish_restart_info();
            return;
        }
        // 获取实际坐标
        real_area.recover_pos.x = center_pos_vec[0].x;
        real_area.recover_pos.y = center_pos_vec[0].y;

        // 获取网格坐标
        grid_area.recover_pos.x = center_pos_vec[0].x / grid_width;
        grid_area.recover_pos.y = center_pos_vec[0].y / grid_height;

        // 获取基地
        std::tie(center_pos_vec, border_rect_vec) = get_area("base", cv_raw_img->image, area_color::base_lower, area_color::base_upper, constant::base_min_width, constant::base_min_height, constant::base_max_width, constant::base_max_height);
        if (center_pos_vec.empty() || border_rect_vec.empty()) {
            RCLCPP_INFO(get_logger(), "failed to get base area!");
            publish_restart_info();
            return;
        }
        // 获取实际坐标
        real_area.base_pos.x = center_pos_vec[0].x;
        real_area.base_pos.y = center_pos_vec[0].y;

        // 获取网格坐标
        grid_area.base_pos.x = center_pos_vec[0].x / grid_width;
        grid_area.base_pos.y = center_pos_vec[0].y / grid_height;

        // 开始获取map
        cv::Mat binary_map; // 存储二值化map

        // 获取迷宫
        cv::inRange(cv_raw_img->image, area_color::map_lower, area_color::map_upper, binary_map);

        // 将处理后的图像处理为map
        // 处理后图像路径是黑色，边缘是白色
        // map.mat存储的是bool，黑色为0，白色为1

        // 对图像进行去噪操作
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::erode(binary_map, binary_map, kernel);
        cv::dilate(binary_map, binary_map, kernel);

        // 获取网格地图
        info_interfaces::msg::Map grid_map;
        grid_map.row = constant::grid_num_h;
        grid_map.col = constant::grid_num_v;
        grid_map.mat.resize(constant::grid_num_h * constant::grid_num_v);
        for (int j = 0; j < constant::grid_num_h; j++) {
            for (int i = 0; i < constant::grid_num_v; i++) {
                grid_map.mat[i + j * constant::grid_num_v] = binary_map.at<uchar>((j + 0.5) * grid_height, (i + 0.5) * grid_width);
            }
        }

        // 获取实际地图
        info_interfaces::msg::Map real_map;
        real_map.row = binary_map.rows;
        real_map.col = binary_map.cols;
        real_map.mat.resize(cv_raw_img->image.rows * cv_raw_img->image.cols);
        for (int j = 0; j < cv_raw_img->image.rows; j++) {
            for (int i = 0; i < cv_raw_img->image.cols; i++) {
                real_map.mat[i + j * cv_raw_img->image.cols] = binary_map.at<uchar>(j, i);
            }
        }

        // 将map发送到寻路模块
        m_grid_map_publisher->publish(grid_map);
        m_real_map_publisher->publish(real_map);
        RCLCPP_INFO(get_logger(), "map has been send!");

        std::shared_ptr<info_interfaces::msg::Map> real_map_ptr = std::make_shared<info_interfaces::msg::Map>(real_map);

        // 获取紫色出入口
        std::tie(center_pos_vec, border_rect_vec) = get_area("purple gate", cv_raw_img->image, area_color::purple_lower, area_color::purple_upper, constant::gate_min_width, constant::gate_min_height, constant::gate_max_width, constant::gate_max_height);
        if (center_pos_vec.empty() || border_rect_vec.empty()) {
            RCLCPP_INFO(get_logger(), "failed to get purple gate!");
            publish_restart_info();
            return;
        }

        // 如果不能直线到达密码发射区，肯定在中心区域外
        if (!algorithm::can_connect(real_map_ptr, center_pos_vec[0].x, center_pos_vec[0].y, real_area.password_pos.x, real_area.password_pos.y)) {
            // 获取实际坐标
            real_area.enter_gate_pos.x = center_pos_vec[0].x;
            real_area.enter_gate_pos.y = center_pos_vec[0].y;

            // 获取网格坐标
            grid_area.enter_gate_pos.x = center_pos_vec[0].x / grid_width;
            grid_area.enter_gate_pos.y = center_pos_vec[0].y / grid_height;
        }
        else {
            // 获取实际坐标
            real_area.exit_gate_pos.x = center_pos_vec[0].x;
            real_area.exit_gate_pos.y = center_pos_vec[0].y;

            // 获取网格坐标
            grid_area.exit_gate_pos.x = center_pos_vec[0].x / grid_width;
            grid_area.exit_gate_pos.y = center_pos_vec[0].y / grid_height;
        }

        // 获取绿色出入口
        std::tie(center_pos_vec, border_rect_vec) = get_area("green gate", cv_raw_img->image, area_color::green_lower, area_color::green_upper, constant::gate_min_width, constant::gate_min_height, constant::gate_max_width, constant::gate_max_height);
        if (center_pos_vec.empty() || border_rect_vec.empty()) {
            RCLCPP_INFO(get_logger(), "failed to get green gate!");
            publish_restart_info();
            return;
        }

        // 如果不能直线到达密码发射区，肯定在中心区域外
        if (!algorithm::can_connect(real_map_ptr, center_pos_vec[0].x, center_pos_vec[0].y, real_area.password_pos.x, real_area.password_pos.y)) {
            // 获取实际坐标
            real_area.enter_gate_pos.x = center_pos_vec[0].x;
            real_area.enter_gate_pos.y = center_pos_vec[0].y;

            // 获取网格坐标
            grid_area.enter_gate_pos.x = center_pos_vec[0].x / grid_width;
            grid_area.enter_gate_pos.y = center_pos_vec[0].y / grid_height;
        }
        else {
            // 获取实际坐标
            real_area.exit_gate_pos.x = center_pos_vec[0].x;
            real_area.exit_gate_pos.y = center_pos_vec[0].y;

            // 获取网格坐标
            grid_area.exit_gate_pos.x = center_pos_vec[0].x / grid_width;
            grid_area.exit_gate_pos.y = center_pos_vec[0].y / grid_height;
        }

        if constexpr (debug_option::print_area_info) {
            RCLCPP_INFO(get_logger(), "enter gate x:%d y:%d", real_area.enter_gate_pos.x, real_area.enter_gate_pos.y);
            RCLCPP_INFO(get_logger(), "exit gate x:%d y:%d", real_area.exit_gate_pos.x, real_area.exit_gate_pos.y);
        }

        m_real_area_publisher->publish(real_area);
        m_grid_area_publisher->publish(grid_area);
        RCLCPP_INFO(get_logger(), "area has been send!");

        // 获取生命值条长度
        std::tie(center_pos_vec, border_rect_vec) = get_area("our robot hp", cv_raw_img->image, area_color::hp_block_lower, area_color::hp_block_upper, constant::hp_min_width, constant::hp_min_height, constant::hp_max_width, constant::hp_max_height);
        if (center_pos_vec.empty() || border_rect_vec.empty()) {
            RCLCPP_INFO(get_logger(), "failed to get our robot hp!");
            publish_restart_info();
            return;
        }

        m_hp_block_width = border_rect_vec[0].width;

        m_initialized = true;
    }
    else {
        info_interfaces::msg::Robot robot;

        // 获取己方机器人位置
        std::tie(center_pos_vec, border_rect_vec) = get_robot("our robot", cv_raw_img->image, area_color::our_robot_lower, area_color::our_robot_upper, constant::our_robot_min_width, constant::our_robot_min_height, constant::our_robot_max_width, constant::our_robot_max_height);
        if (center_pos_vec.empty() || border_rect_vec.empty()) {
            RCLCPP_INFO(get_logger(), "failed to get our robot pos!");
            publish_restart_info();
            m_initialized = false;
            return;
        }
        // 获取实际坐标
        robot.our_robot_real_pos.x = center_pos_vec[0].x;
        robot.our_robot_real_pos.y = center_pos_vec[0].y;

        // 获取网格坐标
        robot.our_robot_grid_pos.x = center_pos_vec[0].x / grid_width;
        robot.our_robot_grid_pos.y = center_pos_vec[0].y / grid_height;

        // 获取己方机器人生命值
        std::tie(center_pos_vec, border_rect_vec) = get_robot("our robot hp", cv_raw_img->image, area_color::hp_block_lower, area_color::hp_block_upper, constant::hp_min_width, constant::hp_min_height, constant::hp_max_width, constant::hp_max_height);
        // m_hp_block_width不能为零，因为要计算生命值百分比
        if (center_pos_vec.empty() || border_rect_vec.empty() || 0 == m_hp_block_width) {
            RCLCPP_INFO(get_logger(), "failed to get our robot current hp!");
            m_initialized = false;
            publish_restart_info();
            return;
        }
        robot.our_robot_hp = static_cast<double>(border_rect_vec[0].width) / static_cast<double>(m_hp_block_width);

        // 获取敌方机器人位置
        std::tie(center_pos_vec, border_rect_vec) = get_robot("enemy robot", cv_raw_img->image, area_color::enemy_lower, area_color::enemy_upper, constant::enemy_min_width, constant::enemy_min_height, constant::enemy_max_width, constant::enemy_max_height);
        if (center_pos_vec.empty() || border_rect_vec.empty()) {
            RCLCPP_INFO(get_logger(), "failed to get enemy robot!");
            m_initialized = false;
            publish_restart_info();
            return;
        }
        for (size_t i = 0; i < center_pos_vec.size(); i++) {
            // 去除不在敌人识别范围内的敌人
            if (center_pos_vec[i].x > constant::enemy_max_x || center_pos_vec[i].y > constant::enemy_max_y) continue;

            info_interfaces::msg::Point enemy_pos;

            // 获取敌人实际坐标
            enemy_pos.x = center_pos_vec[i].x;
            enemy_pos.y = center_pos_vec[i].y;
            robot.enemy_real_pos_vec.push_back(enemy_pos);

            // 获取敌人网格坐标
            enemy_pos.x = center_pos_vec[i].x / grid_width;
            enemy_pos.y = center_pos_vec[i].y / grid_height;
            robot.enemy_grid_pos_vec.push_back(enemy_pos);
        }

        // 将robot发送到寻路模块
        if constexpr (debug_option::print_robot_info) {
            RCLCPP_INFO(get_logger(), "robot has been send!");
        }
        m_robot_publisher->publish(robot);
    }
}

std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> img2map::Node::get_area(const std::string& area_name, cv::InputArray image, cv::Scalar lower, cv::Scalar upper, int min_width, int min_height, int max_width, int max_height)
{
    cv::Mat binary; // 存储二值化图片
    cv::inRange(image, lower, upper, binary);
    // 对图像进行去噪操作
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::erode(binary, binary, kernel);
    cv::dilate(binary, binary, kernel);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours; // 存储轮廓
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::vector<cv::Point> point_vec;
    std::vector<cv::Rect> rect_vec;

    // 遍历每个轮廓
    for (size_t i = 0; i < contours.size(); ++i) {
        // 计算轮廓的矩
        cv::Moments m = cv::moments(contours[i], false);


        // 获取区域的边界矩形
        cv::Rect rect = cv::boundingRect(contours[i]);

        // 去掉不符合大小的
        if (rect.width < min_width || rect.height < min_height || rect.width > max_width || rect.height > max_height) continue;

        rect_vec.push_back(rect);
        // 计算中心位置
        if (m.m00 != 0) {
            int x = static_cast<int>(m.m10 / m.m00);
            int y = static_cast<int>(m.m01 / m.m00);
            if constexpr (debug_option::print_area_info) {
                RCLCPP_INFO(get_logger(), "%s x:%d y:%d w:%d h:%d", area_name.c_str(), x, y, rect.width, rect.height);
            }
            point_vec.push_back(cv::Point(x, y));
        }
    }

    return { point_vec, rect_vec };
}

std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> img2map::Node::get_robot(const std::string& robot_name, cv::InputArray image, cv::Scalar lower, cv::Scalar upper, int min_width, int min_height, int max_width, int max_height)
{
    cv::Mat binary; // 存储二值化图片
    cv::inRange(image, lower, upper, binary);
    // 对图像进行去噪操作
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::erode(binary, binary, kernel);
    cv::dilate(binary, binary, kernel);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours; // 存储轮廓
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::vector<cv::Point> point_vec;
    std::vector<cv::Rect> rect_vec;

    // 遍历每个轮廓
    for (size_t i = 0; i < contours.size(); ++i) {
        // 计算轮廓的矩
        cv::Moments m = cv::moments(contours[i], false);


        // 获取区域的边界矩形
        cv::Rect rect = cv::boundingRect(contours[i]);

        // 去掉不符合大小的
        if (rect.width < min_width || rect.height < min_height || rect.width > max_width || rect.height > max_height) continue;

        rect_vec.push_back(rect);
        // 计算中心位置
        if (m.m00 != 0) {
            int x = static_cast<int>(m.m10 / m.m00);
            int y = static_cast<int>(m.m01 / m.m00);
            if constexpr (debug_option::print_robot_info) {
                RCLCPP_INFO(get_logger(), "%s x:%d y:%d w:%d h:%d", robot_name.c_str(), x, y, rect.width, rect.height);
            }
            point_vec.push_back(cv::Point(x, y));
        }
    }

    return { point_vec, rect_vec };
}

void img2map::Node::publish_restart_info()
{
    m_restart_publisher->publish(m_restart_info);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<img2map::Node>(std::string("img2map_node"));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}