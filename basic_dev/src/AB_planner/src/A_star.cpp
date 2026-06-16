#include "A_star.hpp"

#include <ros/console.h>

AStar::AStar()
{
    
}

AStar::~AStar()
{
}

void AStar::Init(const Eigen::Vector3d& plan_size, double resolution, double reserved_back_space, double safety_distance, double h_weight)
{
    plan_size_ = plan_size;
    resolution_ = resolution;
    reserved_back_space_ = reserved_back_space;
    safety_distance_ = safety_distance;
    h_weight_ = h_weight;
    plan_size_idx_ = Eigen::Vector3i(
        std::round(plan_size_.x() / resolution_),
        std::round(plan_size_.y() / resolution_),
        std::round(plan_size_.z() / resolution_)
    );
    occupancy_grid_.assign(plan_size_idx_.x() * plan_size_idx_.y() * plan_size_idx_.z(), false);

    // 定义26邻域的方向向量
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue; // 跳过自身
                }
                DIRECTIONS_.push_back(Eigen::Vector3i(dx, dy, dz));
            }
        }
    }
}

void AStar::setPlanPoints(const std::vector<Eigen::Vector3d>& points)
{
    need_to_plan_points_ = points;
    if (need_to_plan_points_.empty()) {
        return;
    }

    if (need_to_plan_points_.size() >= 2) {
        const Eigen::Vector3d start = need_to_plan_points_.front();
        const Eigen::Vector3d end = need_to_plan_points_.back();
        go_to_direction_ = (end - start).array().sign().cast<int>(); // 计算起点到终点的方向性，1表示起点坐标小于终点，-1表示起点坐标大于终点，0表示未设置

        Eigen::Vector3d min_delta = Eigen::Vector3d::Zero();
        Eigen::Vector3d max_delta = Eigen::Vector3d::Zero();
        for (const auto& point : need_to_plan_points_) {
            const Eigen::Vector3d delta = point - start;
            min_delta = min_delta.cwiseMin(delta);
            max_delta = max_delta.cwiseMax(delta);
        }

        auto computeOffset = [](double plan_size, double min_delta_axis, double max_delta_axis) {
            const double lower_offset = -min_delta_axis;
            const double upper_offset = plan_size - max_delta_axis;
            const double centered_offset = 0.5 * plan_size;

            if (lower_offset <= upper_offset) {
                return std::clamp(centered_offset, lower_offset, upper_offset);
            }

            // The requested waypoint span is larger than the local grid. Center the
            // window over the span so neither side is systematically discarded.
            return 0.5 * (lower_offset + upper_offset);
        };

        offset_world_to_grid_.x() = static_cast<int>(std::round(computeOffset(plan_size_.x(), min_delta.x(), max_delta.x())));
        offset_world_to_grid_.y() = static_cast<int>(std::round(computeOffset(plan_size_.y(), min_delta.y(), max_delta.y())));
        offset_world_to_grid_.z() = static_cast<int>(std::round(computeOffset(plan_size_.z(), min_delta.z(), max_delta.z())));
    }

    start_point_ = need_to_plan_points_.front(); // 起点位置，单位为世界坐标
}

void AStar::setDynamicObstacles(const std::vector<BoxObstacle>& obstacles)
{
    dynamic_obstacles_ = obstacles;
}

void AStar::setDynamicObstacleParams(double inflation, double endpoint_clear_radius)
{
    dynamic_obstacle_inflation_ = std::max(0.0, inflation);
    endpoint_clear_radius_ = std::max(0.0, endpoint_clear_radius);
}

void AStar::setMaxSearchNodes(size_t max_search_nodes)
{
    max_search_nodes_ = std::max<size_t>(1, max_search_nodes);
}

Eigen::Vector3i AStar::worldToGrid(const Eigen::Vector3d& pos)
{
    // 将世界坐标转换为栅格坐标，单位为米, 将世界坐标系原点转换到栅格坐标系中，并根据分辨率进行缩放，最后四舍五入得到整数栅格坐标
    return Eigen::Vector3i(
        std::round((pos.x() - start_point_.x() + offset_world_to_grid_.x()) / resolution_),
        std::round((pos.y() - start_point_.y() + offset_world_to_grid_.y()) / resolution_),
        std::round((pos.z() - start_point_.z() + offset_world_to_grid_.z()) / resolution_)
    );
}

Eigen::Vector3d AStar::gridToWorld(const Eigen::Vector3i& idx)
{
    // 将栅格坐标转换为世界坐标，单位为米, 根据分辨率进行缩放，并将栅格坐标系原点转换到世界坐标系中
    return Eigen::Vector3d(
        idx.x() * resolution_ - offset_world_to_grid_.x() + start_point_.x(),
        idx.y() * resolution_ - offset_world_to_grid_.y() + start_point_.y(),
        idx.z() * resolution_ - offset_world_to_grid_.z() + start_point_.z()
    );
}

int64_t AStar::idxToKey(const Eigen::Vector3i& idx)
{
    // 将栅格坐标转换为唯一的整数键值，方便在哈希表中存储和查找
    return static_cast<int64_t>(idx.x()) * plan_size_idx_.y() * plan_size_idx_.z() +
           static_cast<int64_t>(idx.y()) * plan_size_idx_.z() +
           static_cast<int64_t>(idx.z());
}

bool AStar::isBounds(const Eigen::Vector3i& idx)
{
    // 判断栅格坐标是否在规划范围内
    return (idx.x() >= 0 && idx.x() < plan_size_idx_.x() &&
            idx.y() >= 0 && idx.y() < plan_size_idx_.y() &&
            idx.z() >= 0 && idx.z() < plan_size_idx_.z());
}

bool AStar::isOccupied(const Eigen::Vector3i& idx, double safety_distance)
{
    // 判断栅格坐标是否被占用，考虑安全距离，单位为米, 需要将安全距离转换为栅格单位，并在占用判断时考虑周围的栅格
    int safety_cells = std::ceil(safety_distance / resolution_);
    for (int dx = -safety_cells; dx <= safety_cells; ++dx) {
        for (int dy = -safety_cells; dy <= safety_cells; ++dy) {
            for (int dz = -safety_cells; dz <= safety_cells; ++dz) {
                Eigen::Vector3i neighbor_idx = idx + Eigen::Vector3i(dx, dy, dz);
                if (isBounds(neighbor_idx)) {
                    if (occupancy_grid_[idxToKey(neighbor_idx)]) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

double AStar::getHeuristic(const Eigen::Vector3i& idx1, const Eigen::Vector3i& idx2)
{
    // 计算启发式代价，使用欧几里得距离作为启发式函数，单位为米
    return h_weight_ * (gridToWorld(idx1) - gridToWorld(idx2)).norm();
}

void AStar::clearOccupancyGrid()
{
    std::fill(occupancy_grid_.begin(), occupancy_grid_.end(), false);
}

void AStar::updateOccupancyGrid()
{
    clearOccupancyGrid();
    for (const auto& obstacle : dynamic_obstacles_) {
        markObstacleBox(obstacle);
    }

    for (const auto& point : need_to_plan_points_) {
        clearAroundPoint(point, endpoint_clear_radius_);
    }
}

void AStar::markObstacleBox(const BoxObstacle& obstacle)
{
    if (!std::isfinite(obstacle.center.x()) || !std::isfinite(obstacle.center.y()) || !std::isfinite(obstacle.center.z()) ||
        !std::isfinite(obstacle.size.x()) || !std::isfinite(obstacle.size.y()) || !std::isfinite(obstacle.size.z())) {
        return;
    }

    const Eigen::Vector3d half_size = 0.5 * obstacle.size.cwiseMax(Eigen::Vector3d::Zero()) +
                                      Eigen::Vector3d::Constant(dynamic_obstacle_inflation_);
                                      
    Eigen::Matrix3d R = obstacle.orientation.toRotationMatrix();
    Eigen::Vector3d ex = R.col(0) * half_size.x();
    Eigen::Vector3d ey = R.col(1) * half_size.y();
    Eigen::Vector3d ez = R.col(2) * half_size.z();
    
    double bound_x = std::abs(ex.x()) + std::abs(ey.x()) + std::abs(ez.x());
    double bound_y = std::abs(ex.y()) + std::abs(ey.y()) + std::abs(ez.y());
    double bound_z = std::abs(ex.z()) + std::abs(ey.z()) + std::abs(ez.z());
    
    Eigen::Vector3d bounds(bound_x, bound_y, bound_z);

    const Eigen::Vector3i min_idx = worldToGrid(obstacle.center - bounds);
    const Eigen::Vector3i max_idx = worldToGrid(obstacle.center + bounds);

    const int min_x = std::max(0, std::min(min_idx.x(), max_idx.x()));
    const int min_y = std::max(0, std::min(min_idx.y(), max_idx.y()));
    const int min_z = std::max(0, std::min(min_idx.z(), max_idx.z()));
    const int max_x = std::min(plan_size_idx_.x() - 1, std::max(min_idx.x(), max_idx.x()));
    const int max_y = std::min(plan_size_idx_.y() - 1, std::max(min_idx.y(), max_idx.y()));
    const int max_z = std::min(plan_size_idx_.z() - 1, std::max(min_idx.z(), max_idx.z()));

    if (min_x > max_x || min_y > max_y || min_z > max_z) {
        return;
    }

    Eigen::Matrix3d R_inv = R.transpose();
    for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {
            for (int z = min_z; z <= max_z; ++z) {
                Eigen::Vector3d pt = gridToWorld(Eigen::Vector3i(x, y, z));
                Eigen::Vector3d diff = R_inv * (pt - obstacle.center);
                if (std::abs(diff.x()) <= half_size.x() &&
                    std::abs(diff.y()) <= half_size.y() &&
                    std::abs(diff.z()) <= half_size.z()) {
                    occupancy_grid_[idxToKey(Eigen::Vector3i(x, y, z))] = true;
                }
            }
        }
    }
}

void AStar::clearAroundPoint(const Eigen::Vector3d& point, double radius)
{
    if (radius <= 0.0 || occupancy_grid_.empty()) {
        return;
    }

    const Eigen::Vector3i center_idx = worldToGrid(point);
    const int clear_cells = std::ceil(radius / resolution_);
    for (int dx = -clear_cells; dx <= clear_cells; ++dx) {
        for (int dy = -clear_cells; dy <= clear_cells; ++dy) {
            for (int dz = -clear_cells; dz <= clear_cells; ++dz) {
                const Eigen::Vector3i idx = center_idx + Eigen::Vector3i(dx, dy, dz);
                if (!isBounds(idx)) {
                    continue;
                }
                if ((gridToWorld(idx) - point).norm() <= radius) {
                    occupancy_grid_[idxToKey(idx)] = false;
                }
            }
        }
    }
}

bool AStar::planPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, std::vector<Eigen::Vector3d>& path)
{
    // A*路径规划算法的实现，输入起点和终点的世界坐标，输出路径点的世界坐标列表
    // 1. 将起点和终点转换为栅格坐标
    // 2. 初始化开放列表->std::priority_queue, 用一维std::unordered_map存储栅格坐标到节点指针的映射，初始化关闭列表->在节点结构体中添加is_closed成员变量
    // 3. 将起点加入开放列表，设置g_cost为0，计算h_cost和f_cost
    // 4. 循环直到开放列表为空
    //    a. 从开放列表中取出f_cost最小的节点作为当前节点
    //    b. 如果当前节点是终点，则从当前节点回溯到起点得到路径，并返回路径点的世界坐标列表
    //    c. 将当前节点加入关闭列表
    //    d. 对当前节点的26邻域进行遍历，对于每个邻居节点：
    //       i. 如果邻居节点在关闭列表中，则跳过
    //       ii. 如果邻居节点不可达（越界或占用），则跳过
    //       iii. 计算从起点到邻居节点的g_cost，如果邻居节点不在开放列表中，则将其加入开放列表，并设置g_cost、h_cost和f_cost，以及父节点指针；如果邻居节点已经在开放列表中，但新的g_cost更小，则更新其g_cost、f_cost和父节点指针
    // 5. 如果循环结束后仍未找到路径，则返回空路径
    path.clear();
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
    std::unordered_map<int64_t, AStarNode> node_map;
    Eigen::Vector3i start_idx = worldToGrid(start);
    Eigen::Vector3i goal_idx = worldToGrid(goal);

    if (!isBounds(start_idx) || !isBounds(goal_idx)) {
        ROS_WARN("A* start or goal is out of local grid bounds. start_idx=(%d,%d,%d), goal_idx=(%d,%d,%d), plan_size_idx=(%d,%d,%d).",
                 start_idx.x(), start_idx.y(), start_idx.z(),
                 goal_idx.x(), goal_idx.y(), goal_idx.z(),
                 plan_size_idx_.x(), plan_size_idx_.y(), plan_size_idx_.z());
        return false;
    }

    const size_t search_budget = std::max<size_t>(1, max_search_nodes_);
    node_map.reserve(std::min(search_budget + 1, static_cast<size_t>(occupancy_grid_.size())));
    
    auto createNode = [&](const Eigen::Vector3i& idx) -> AStarNode& {
        int64_t key = idxToKey(idx);
        if (node_map.find(key) == node_map.end()) {
            AStarNode new_node;
            new_node.idx = idx;
            node_map[key] = new_node;
        }
        return node_map[key];
    };

    AStarNode& start_node = createNode(start_idx);
    start_node.g_cost = 0.0;
    start_node.h_cost = getHeuristic(start_idx, goal_idx);
    start_node.f_cost = start_node.h_cost;

    bool path_found = false;
    open_list.push(start_node);
    size_t expanded_nodes = 0;

    while(open_list.empty() == false) {
        AStarNode current_node = open_list.top();
        open_list.pop();
        AStarNode& current_node_ref = createNode(current_node.idx); // 获取当前节点的引用，方便修改其成员变量
        if (current_node_ref.is_closed) {
            continue; // 如果当前节点已经在关闭列表中，则跳过
        }
        const Eigen::Vector3i current_idx = current_node_ref.idx;
        const double current_g_cost = current_node_ref.g_cost;
        bool has_previous_direction = false;
        Eigen::Vector3i previous_direction = Eigen::Vector3i::Zero();
        if (current_node_ref.has_parent) {
            auto parent_it = node_map.find(current_node_ref.parent_key);
            if (parent_it != node_map.end()) {
                previous_direction = current_idx - parent_it->second.idx;
                has_previous_direction = true;
            }
        }

        if (current_idx == goal_idx) {
            path_found = true;
            const AStarNode* trace_node = &current_node_ref;
            while (trace_node != nullptr) {
                path.push_back(gridToWorld(trace_node->idx));
                if (!trace_node->has_parent) {
                    break;
                }
                auto parent_it = node_map.find(trace_node->parent_key);
                if (parent_it == node_map.end()) {
                    ROS_WARN("A* path reconstruction failed because parent node is missing.");
                    path.clear();
                    path_found = false;
                    break;
                }
                trace_node = &parent_it->second;
            }
            std::reverse(path.begin(), path.end()); // 将路径反转，使其从起点到终点
            break;
        }
        current_node_ref.is_closed = true; // 将当前节点加入关闭列表
        expanded_nodes++;
        if (expanded_nodes > search_budget) {
            ROS_WARN("A* search aborted after expanding %lu nodes without reaching goal. start=(%.2f, %.2f, %.2f), goal=(%.2f, %.2f, %.2f), budget=%lu.",
                     static_cast<unsigned long>(expanded_nodes),
                     start.x(), start.y(), start.z(),
                     goal.x(), goal.y(), goal.z(),
                     static_cast<unsigned long>(search_budget));
            return false;
        }

        // 对当前节点的26邻域进行遍历
        for (const auto& direction : DIRECTIONS_) {
            Eigen::Vector3i neighbor_idx = current_idx + direction;
            if (isBounds(neighbor_idx) == false) {
                continue; // 如果邻居节点越界，则跳过
            }
            if (isOccupied(neighbor_idx, safety_distance_) == true) {
                continue; // 如果邻居节点不可达（占用），则跳过
            }
            const double step_cost = resolution_ * direction.cast<double>().norm(); // 对角线移动按真实距离计费，避免过度偏好大对角线

            double line_deviation_cost = 0.0;
            const Eigen::Vector3d start_world = gridToWorld(start_idx);
            const Eigen::Vector3d goal_world = gridToWorld(goal_idx);
            const Eigen::Vector3d neighbor_world = gridToWorld(neighbor_idx);
            const Eigen::Vector3d start_to_goal = goal_world - start_world;
            const double start_to_goal_len_sq = start_to_goal.squaredNorm();
            if (start_to_goal_len_sq > 1e-6) {
                const double progress = std::clamp((neighbor_world - start_world).dot(start_to_goal) / start_to_goal_len_sq, 0.0, 1.0);
                const Eigen::Vector3d projected = start_world + progress * start_to_goal;
                line_deviation_cost = 0.05 * (neighbor_world - projected).norm();
            }

            double turn_cost = 0.0;
            if (has_previous_direction) {
                if (previous_direction != direction) {
                    turn_cost = 0.25 * resolution_;
                }
            }

            double tentative_g_cost = current_g_cost + step_cost + line_deviation_cost + turn_cost;
            AStarNode& neighbor_node = createNode(neighbor_idx);
            if (neighbor_node.is_closed) {
                continue; // 如果邻居节点已经在关闭列表中，则跳过
            }
            if (tentative_g_cost < neighbor_node.g_cost) {
                neighbor_node.parent_key = idxToKey(current_idx); // 更新父节点
                neighbor_node.has_parent = true;
                neighbor_node.g_cost = tentative_g_cost; // 更新g_cost
                neighbor_node.h_cost = getHeuristic(neighbor_idx, goal_idx); // 更新h_cost
                neighbor_node.f_cost = neighbor_node.g_cost + neighbor_node.h_cost; // 更新f_cost
                open_list.push(neighbor_node); // 将邻居节点加入开放列表
            }
        }
    }
    return path_found;
}

bool AStar::planPaths(const std::vector<Eigen::Vector3d>& waypoints, std::vector<std::vector<Eigen::Vector3d>>& paths)
{
    // 对多个航路点进行路径规划，输入航路点的世界坐标列表，输出每对连续航路点之间的路径点的世界坐标列表
    paths.clear();
    if (waypoints.size() < 2) {
        return false; // 如果航路点数量不足以形成路径，则返回失败
    }

    // 设置需要规划的点列表，包含起点和终点和中间的航路点
    setPlanPoints(waypoints);
    updateOccupancyGrid();

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        std::vector<Eigen::Vector3d> path;
        bool success = planPath(waypoints[i], waypoints[i + 1], path);
        if (success == false) {
            return false; // 如果任意一对连续航路点之间的路径规划失败，则返回失败
        }
        paths.push_back(path); // 将成功规划的路径加入结果列表
    }
    return true; // 所有路径规划成功，返回成功
}
