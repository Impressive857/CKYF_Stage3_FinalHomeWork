#ifndef _IMG2MAP_NODE_HPP_
#define _IMG2MAP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/msg/image.hpp"
#include "info_interfaces/msg/area.hpp"
#include "info_interfaces/msg/robot.hpp"
#include "info_interfaces/msg/map.hpp"
#include "example_interfaces/msg/bool.hpp"

#include "constant.hpp"

namespace img2map {
    class Node
        :public rclcpp::Node
    {
    public:
        explicit Node(const std::string& name);
    private:
        void img2map_cbfn(const sensor_msgs::msg::Image::SharedPtr ros_img);
        /// @brief 获取图片中指定颜色区域
        /// @param image 要获取的图片
        /// @param lower 颜色获取范围下限
        /// @param upper 颜色获取范围上限
        /// @return 指定颜色区域中心坐标vector和边界矩形vector
        std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> get_area(const std::string& area_name, cv::InputArray image, cv::Scalar lower, cv::Scalar upper, int min_width, int min_height, int max_width, int max_height);
        std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> get_robot(const std::string& robot_name, cv::InputArray image, cv::Scalar lower, cv::Scalar upper, int min_width, int min_height, int max_width, int max_height);
        void publish_restart_info();
    private:
        bool m_initialized;
        int m_hp_block_width; // 生命值条宽度
        example_interfaces::msg::Bool m_restart_info;
        rclcpp::Publisher<info_interfaces::msg::Map>::SharedPtr m_map_publisher;
        rclcpp::Publisher<info_interfaces::msg::Area>::SharedPtr m_area_publisher;
        rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr m_restart_publisher;
        rclcpp::Publisher<info_interfaces::msg::Robot>::SharedPtr m_robot_publisher;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_img_subscription;
    };
}

#endif // ^^ !_IMG2MAP_NODE_HPP_