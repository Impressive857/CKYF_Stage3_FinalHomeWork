#ifndef _NAVIGATION_NODE_HPP_
#define _NAVIGATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <queue>

#include <geometry_msgs/msg/pose.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/int64.hpp>

#include "info_interfaces/msg/map.hpp"
#include "info_interfaces/msg/area.hpp"
#include "info_interfaces/msg/robot.hpp"

#include "constant.hpp"

namespace navigation {
    namespace algorithm {
        struct Node {
            int x;
            int y;
            int real_cost;
            int heuristic_cost;
            Node* parent;
            Node(int x, int y, int real_cost, int heuristic_cost, Node* parent = nullptr)
            {
                this->x = x;
                this->y = y;
                this->real_cost = real_cost;
                this->heuristic_cost = heuristic_cost;
                this->parent = parent;
            }
            int total_cost() const {
                return this->real_cost + this->heuristic_cost;
            }
            bool operator<(const Node& other) const {
                // 这里反向重载是为了让优先队列将代价小的放在顶部
                return this->total_cost() > other.total_cost();
            }
        };
        int manhattan_distance(int x1, int y1, int x2, int y2) {
            return std::abs(x1 - x2) + std::abs(y1 - y2);
        }
        using Path = std::vector<std::pair<int, int>>;
        Path a_star(info_interfaces::msg::Map map, int src_x, int src_y, int dst_x, int dst_y) {
            std::priority_queue<Node> to_visit;
            std::unordered_map<int, Node*> visited;
            std::unordered_set<Node*> node_allocated;

            Node start(src_x, src_y, 0, manhattan_distance(src_x, src_y, dst_x, dst_y));
            to_visit.push(start);

            while (!to_visit.empty()) {
                Node* current = new Node(to_visit.top().x, to_visit.top().y, to_visit.top().real_cost, to_visit.top().heuristic_cost, to_visit.top().parent);
                node_allocated.insert(current);
                visited[current->x * map.col + current->y] = current;
                to_visit.pop();

                if (current->x == dst_x && current->y == dst_y) {
                    Path path;
                    Node* node = current;
                    while (nullptr != node) {
                        path.push_back({ node->x, node->y });
                        node = node->parent;
                    }
                    std::reverse(path.begin(), path.end());
                    for (Node* node : node_allocated) {
                        delete node;
                    }
                    return path;
                }

                for (int dx = -1; dx <= 1;dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        // 跳过当前节点
                        if (dx == 0 && dy == 0) continue;

                        int new_x = current->x + dx;
                        int new_y = current->y + dy;

                        if (new_x > 0 && new_x < map.row && new_y >0 && new_y < map.col && 0 == map.mat[new_x * map.col + new_y]) {
                            int new_real_cost = current->real_cost + 1;
                            int new_heuristic_cost = manhattan_distance(new_x, new_y, dst_x, dst_y);
                            Node* neighbor = new Node(new_x, new_y, new_real_cost, new_heuristic_cost, current);

                            // 如果邻居节点已经探索过，并且新的实际代价更大，则跳过
                            if (visited.find(new_x * map.col + new_y) != visited.end() && visited[new_x * map.col + new_y]->real_cost <= new_real_cost) {
                                delete neighbor;
                                continue;
                            }
                            node_allocated.insert(neighbor);
                            to_visit.push(*neighbor);
                        }
                    }
                }
            }

            for (Node* node : node_allocated) {
                delete node;
            }

            // 没有找到路径则返回空
            return {};
        }
    };
    class Node
        : public rclcpp::Node
    {
    public:
        explicit Node(const std::string& name);
    private:
        void map_init_cbfn(const info_interfaces::msg::Map::SharedPtr map_info);
        void area_init_cbfn(const info_interfaces::msg::Area::SharedPtr area_info);
        void robot_navigation_cbfn(const info_interfaces::msg::Robot::SharedPtr robot_info);
        void password_cbfn(const example_interfaces::msg::Int64::SharedPtr password);
        void password_segment_cbfn(const example_interfaces::msg::Int64::SharedPtr password_segment);
    private:
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_our_pose_publisher;
        rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr m_shoot_publisher;
        rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr m_password_subscription;
        rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr m_password_segment_subscription;
        rclcpp::Subscription<info_interfaces::msg::Map>::SharedPtr m_map_subscription;
        rclcpp::Subscription<info_interfaces::msg::Area>::SharedPtr m_area_subscription;
        rclcpp::Subscription<info_interfaces::msg::Robot>::SharedPtr m_robot_subscription;
        info_interfaces::msg::Area m_area;
        info_interfaces::msg::Map m_map;
        example_interfaces::msg::Int64 m_password;
        std::vector<example_interfaces::msg::Int64> m_password_segment_vec;
    };
}

#endif // ^^ !_NAVIGATION_NODE_HPP_