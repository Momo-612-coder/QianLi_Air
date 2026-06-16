#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <vector>
#include "geometry_msgs/Point.h"

const double drone_reach_threshold = 4.0; // 定义一个距离阈值，当无人机与目标点的距离小于该值时认为到达目标点
const double point_and_point_distance_threshold = 10.0; // 定义一个距离阈值，当两个路径点之间的距离小于该值时认为它们是相同的，只取其中一个
const double max_drone_and_point_distance_threshold = 100.0; // 定义一个距离阈值，当无人机与路径点的距离大于该值时认为路径点不可达，应该被移除

const int min_similar_observations = 3; // 同类观测累计达到该次数后，才加入实际路径

class PathPlanner
{
public:
    PathPlanner();
    ~PathPlanner();

    bool is_drone_reach_target(const geometry_msgs::Point& drone_position, const geometry_msgs::Point& target_point);
    void update_drone_position(const geometry_msgs::Point& drone_position);
    bool add_observation(const geometry_msgs::Point& observed_point);
    bool pop_reached_target();
    bool has_target() const;
    geometry_msgs::Point current_target() const;

    std::vector<geometry_msgs::Point> target_path_;
    geometry_msgs::Point current_drone_position_;

private:
    struct CandidatePoint
    {
        geometry_msgs::Point point;
        int hit_count = 0;
    };

    std::vector<CandidatePoint> candidate_points_;

    double point_distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) const;
    void sort_path_by_distance();
    void prune_unreachable_points();
    void merge_or_insert_target_point(const geometry_msgs::Point& point);

};

#endif