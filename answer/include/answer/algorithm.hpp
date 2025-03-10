#ifndef _ALGORITHM_HPP_
#define _ALGORITHM_HPP_

#include <queue>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include "info_interfaces/msg/map.hpp"

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
    };
    struct compare_node {
        bool operator()(const Node* left, const Node* right) const {
            // 这里是为了让优先队列将代价小的放在顶部
            return left->total_cost() > right->total_cost();
        }
    };
    int manhattan_distance(int x1, int y1, int x2, int y2) {
        return std::abs(x1 - x2) + std::abs(y1 - y2);
    }
    double euclidean_distance(int x1, int y1, int x2, int y2) {
        return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }
    using Path = std::vector<std::pair<int, int>>;
    Path a_star(const info_interfaces::msg::Map::SharedPtr map, int src_x, int src_y, int dst_x, int dst_y) {
        std::priority_queue<Node*, std::vector<Node*>, compare_node> to_visit;
        std::unordered_map<int, Node*> to_visit_map;
        std::unordered_map<int, Node*> visited;
        std::unordered_set<Node*> node_allocated;

        Node* start = new Node(src_x, src_y, 0, manhattan_distance(src_x, src_y, dst_x, dst_y));
        to_visit_map[src_y * map->col + src_x] = start;
        to_visit.push(start);
        node_allocated.insert(start);

        while (!to_visit.empty()) {
            Node* current = to_visit.top();
            visited[current->y * map->col + current->x] = current;
            to_visit.pop();
            to_visit_map.erase(current->y * map->col + current->x);

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

            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    // 跳过当前节点
                    if (0 == dx && 0 == dy) continue;

                    int new_x = current->x + dx;
                    int new_y = current->y + dy;

                    if (new_x >= 0 && new_x < static_cast<int>(map->col) && new_y >= 0 && new_y < static_cast<int>(map->row) && 0 == map->mat[new_y * map->col + new_x]) {
                        // 如果已经遍历过，则跳过
                        if (visited.find(new_y * map->col + new_x) != visited.end()) continue;

                        Node* neighbor = nullptr;
                        // 在待遍历列表中寻找邻居
                        if (to_visit_map.find(new_y * map->col + new_x) != to_visit_map.end()) {
                            neighbor = to_visit_map[new_y * map->col + new_x];
                        }

                        int new_real_cost = current->real_cost + 1;
                        int new_heuristic_cost = manhattan_distance(new_x, new_y, dst_x, dst_y);

                        // 如果没找到，则新建一个
                        if (nullptr == neighbor) {
                            neighbor = new Node(new_x, new_y, new_real_cost, new_heuristic_cost, current);
                            node_allocated.insert(neighbor);
                            to_visit.push(neighbor);
                            to_visit_map.insert({ new_y * map->col + new_x, neighbor });
                        }
                        // 如果找到，并且新的实际代价更低，则更新
                        else if (neighbor->real_cost > new_real_cost) {
                            neighbor->real_cost = new_real_cost;
                            neighbor->heuristic_cost = new_heuristic_cost;
                            neighbor->parent = current;
                        }
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

    bool can_connect(const info_interfaces::msg::Map::SharedPtr map, int src_x, int src_y, int dst_x, int dst_y) {
        if (src_x < 0 || src_x >= static_cast<int>(map->col) || src_y < 0 || src_y >= static_cast<int>(map->row)
            ||
            dst_x < 0 || dst_x >= static_cast<int>(map->col) || dst_y < 0 || dst_y >= static_cast<int>(map->row)
            ) {
            return false;
        }

        // 计算两点之间的距离
        int dx = dst_x - src_x;
        int dy = dst_y - src_y;
        int steps = std::max(std::abs(dx), std::abs(dy));

        // 计算每一步的增量
        double xIncrement = static_cast<double>(dx) / steps;
        double yIncrement = static_cast<double>(dy) / steps;

        // 遍历中间点
        for (int i = 1; i < steps; ++i) {
            double x = src_x + i * xIncrement;
            double y = src_y + i * yIncrement;

            // 取整得到中间点的坐标
            int ix = static_cast<int>(std::round(x));
            int iy = static_cast<int>(std::round(y));

            // 检查中间点是否越界或为障碍物
            if (ix < 0 || ix >= static_cast<int>(map->col) || iy < 0 || iy >= static_cast<int>(map->row) || map->mat[iy * map->col + ix] != 0) {
                return false;
            }
        }

        return true;
    }
};

#endif // !_ALGORITHM_HPP_