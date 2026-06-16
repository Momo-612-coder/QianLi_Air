#include "path_planner.hpp"

#include <algorithm>
#include <cmath>

PathPlanner::PathPlanner()
{

}

PathPlanner::~PathPlanner()
{

}

double PathPlanner::point_distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) const
{
	const double dx = a.x - b.x;
	const double dy = a.y - b.y;
	const double dz = a.z - b.z;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool PathPlanner::is_drone_reach_target(const geometry_msgs::Point& drone_position, const geometry_msgs::Point& target_point)
{
	return point_distance(drone_position, target_point) <= drone_reach_threshold;
}

void PathPlanner::sort_path_by_distance()
{
	std::sort(target_path_.begin(), target_path_.end(),
			  [this](const geometry_msgs::Point& a, const geometry_msgs::Point& b)
			  {
				  return point_distance(current_drone_position_, a) < point_distance(current_drone_position_, b);
			  });
}

void PathPlanner::prune_unreachable_points()
{
	target_path_.erase(
		std::remove_if(target_path_.begin(), target_path_.end(),
					   [this](const geometry_msgs::Point& p)
					   {
						   return point_distance(current_drone_position_, p) > max_drone_and_point_distance_threshold;
					   }),
		target_path_.end());
}

void PathPlanner::merge_or_insert_target_point(const geometry_msgs::Point& point)
{
	for (auto& p : target_path_)
	{
		if (point_distance(p, point) <= point_and_point_distance_threshold)
		{
			p.x = 0.5 * (p.x + point.x);
			p.y = 0.5 * (p.y + point.y);
			p.z = 0.5 * (p.z + point.z);
			return;
		}
	}

	target_path_.push_back(point);
}

void PathPlanner::update_drone_position(const geometry_msgs::Point& drone_position)
{
	current_drone_position_ = drone_position;
	pop_reached_target();
	prune_unreachable_points();
	sort_path_by_distance();
}

bool PathPlanner::add_observation(const geometry_msgs::Point& observed_point)
{
	for (auto it = candidate_points_.begin(); it != candidate_points_.end(); ++it)
	{
		if (point_distance(it->point, observed_point) <= point_and_point_distance_threshold)
		{
			const int old_hits = it->hit_count;
			const int new_hits = old_hits + 1;

			// 用命中次数做加权均值，让候选点更稳定。
			it->point.x = (it->point.x * old_hits + observed_point.x) / new_hits;
			it->point.y = (it->point.y * old_hits + observed_point.y) / new_hits;
			it->point.z = (it->point.z * old_hits + observed_point.z) / new_hits;
			it->hit_count = new_hits;

			if (it->hit_count >= min_similar_observations)
			{
				merge_or_insert_target_point(it->point);
				candidate_points_.erase(it);
				prune_unreachable_points();
				sort_path_by_distance();
				return true;
			}

			return false;
		}
	}

	CandidatePoint candidate;
	candidate.point = observed_point;
	candidate.hit_count = 1;
	candidate_points_.push_back(candidate);
	return false;
}

bool PathPlanner::pop_reached_target()
{
	bool popped = false;
	while (!target_path_.empty() && is_drone_reach_target(current_drone_position_, target_path_.front()))
	{
		target_path_.erase(target_path_.begin());
		popped = true;
	}
	return popped;
}

bool PathPlanner::has_target() const
{
	return !target_path_.empty();
}

geometry_msgs::Point PathPlanner::current_target() const
{
	if (!target_path_.empty())
	{
		return target_path_.front();
	}

	geometry_msgs::Point empty_point;
	empty_point.x = 0.0;
	empty_point.y = 0.0;
	empty_point.z = 0.0;
	return empty_point;
}