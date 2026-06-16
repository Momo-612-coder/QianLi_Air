#ifndef __A_STAR_HPP_
#define __A_STAR_HPP_

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include "Eigen/Dense"

// 以NED坐标系，z轴向下，单位为米
// 栅格坐标系

struct AStarNode
{
    Eigen::Vector3i idx; // 节点位置 -- 栅格坐标
    double g_cost = 1e9; // 从起点到当前节点的实际代价
    double h_cost = 1e9; // 从当前节点到目标节点的启发式估计代价
    double f_cost = 1e9; // g_cost + h_cost
    int64_t parent_key = -1; // 父节点在哈希表中的key，避免unordered_map扩容后指针失效
    bool has_parent = false; // 是否存在父节点
    bool is_closed = false; // 是否在关闭列表中

    // 为优先队列定义比较运算符，f_cost较小的节点优先
    bool operator>(const AStarNode& other) const
    {
        return f_cost > other.f_cost;
    }
};

struct BoxObstacle
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d size = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

class AStar
{
public:
    AStar();
    ~AStar();

    void Init(const Eigen::Vector3d& plan_size, double resolution, double reserved_back_space, double safety_distance, double h_weight);
    void setPlanPoints(const std::vector<Eigen::Vector3d>& points);
    void setDynamicObstacles(const std::vector<BoxObstacle>& obstacles);
    void setDynamicObstacleParams(double inflation, double endpoint_clear_radius);
    void setMaxSearchNodes(size_t max_search_nodes);
    Eigen::Vector3i worldToGrid(const Eigen::Vector3d& pos);
    Eigen::Vector3d gridToWorld(const Eigen::Vector3i& idx);
    int64_t idxToKey(const Eigen::Vector3i& idx);
    bool isBounds(const Eigen::Vector3i& idx);
    bool isOccupied(const Eigen::Vector3i& idx, double safety_distance);
    double getHeuristic(const Eigen::Vector3i& idx1, const Eigen::Vector3i& idx2);
    void clearOccupancyGrid();
    void updateOccupancyGrid();
    void markObstacleBox(const BoxObstacle& obstacle);
    void clearAroundPoint(const Eigen::Vector3d& point, double radius);

    bool planPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, std::vector<Eigen::Vector3d>& path);
    bool planPaths(const std::vector<Eigen::Vector3d>& waypoints, std::vector<std::vector<Eigen::Vector3d>>& paths);

    // 类参数
    Eigen::Vector3d plan_size_;
    Eigen::Vector3i plan_size_idx_;
    double resolution_;
    double reserved_back_space_;
    double safety_distance_;
    double h_weight_;
    std::vector<Eigen::Vector3i> DIRECTIONS_; // 26邻域定义

    Eigen::Vector3i go_to_direction_ = Eigen::Vector3i::Zero(); // 起点到终点的方向性，1表示起点坐标小于终点，-1表示起点坐标大于终点，0表示未设置
    Eigen::Vector3i offset_world_to_grid_ = Eigen::Vector3i::Zero(); // 世界坐标系原点在栅格坐标系中的偏移，单位为米
    Eigen::Vector3d start_point_; // 起点位置，单位为世界坐标
    Eigen::Vector3i start_idx_; // 起点位置，单位为栅格坐标
    std::vector<Eigen::Vector3d> need_to_plan_points_; // 需要规划的点列表，单位为世界坐标，包含起点和终点和中间的航路点
    std::vector<BoxObstacle> dynamic_obstacles_; // 动态障碍物盒，单位为世界坐标
    double dynamic_obstacle_inflation_ = 2.5; // 动态障碍物膨胀半径，单位为米
    double endpoint_clear_radius_ = 2.0; // 起点/目标点附近强制清空半径，单位为米
    size_t max_search_nodes_ = 500000; // 单次A*最多展开节点数，防止失败场景阻塞ROS回调

    std::vector<bool> occupancy_grid_; // 占用栅格地图，true表示占用，false表示空闲

};

#endif
