#ifndef __B_SPLINE_HPP_
#define __B_SPLINE_HPP_

#include <vector>
#include <ros/ros.h>
#include "Eigen/Dense"
#include "iostream"
#include <cmath>
#include <algorithm>
#include <limits>



class BSpline
{
public:
    BSpline();
    ~BSpline();

    void setControlPoints(const std::vector<Eigen::Vector3d>& control_points);
    int numSegments() const;
    double maxU() const;
    std::pair<int, double> normalizeU(double u) const;
    static Eigen::RowVector4d basisFunctions(double dt);
    static Eigen::RowVector4d derivativeBasis(double dt, int m);
    Eigen::Matrix<double, 4, 3> segmentCtrlPoints(int k) const;

    Eigen::Vector3d position(double u) const;
    Eigen::Vector3d velocity(double u) const;
    Eigen::Vector3d acceleration(double u) const;

    std::vector<Eigen::Vector3d> control_points_;
    double knot_interval_ = 1.0; // 默认均匀节点间隔
    int degree_ = 3; // 默认三阶B样条

};

class ArcLengthTable
{
public:
    void build(const BSpline& bspline, double u_start, double u_end, int num_points = 500);
    double getLength(double u) const;
    double getUfromS(double s) const;
    double getSfromU(double u) const;

    // ===== 最近点投影（牛顿法） =====
    inline double findClosestU(const Eigen::Vector3d& pos,
                           const BSpline& spline,
                           double u_guess,
                           double u_min, double u_max,
                           int max_iter = 10)
    {
        double u = std::clamp(u_guess, u_min, u_max);

        if (!u_table_.empty()) {
            double best_dist_sq = std::numeric_limits<double>::infinity();
            double best_u = u;
            for (const double candidate_u : u_table_) {
                const double clamped_candidate = std::clamp(candidate_u, u_min, u_max);
                const double dist_sq = (spline.position(clamped_candidate) - pos).squaredNorm();
                if (dist_sq < best_dist_sq) {
                    best_dist_sq = dist_sq;
                    best_u = clamped_candidate;
                }
            }

            if ((spline.position(best_u) - pos).squaredNorm() + 1e-6 <
                (spline.position(u) - pos).squaredNorm()) {
                u = best_u;
            }
        }

        for (int iter = 0; iter < max_iter; ++iter) {
            const Eigen::Vector3d p = spline.position(u);
            const Eigen::Vector3d v = spline.velocity(u);
            const Eigen::Vector3d a = spline.acceleration(u);
            const Eigen::Vector3d r = p - pos;

            const double grad = r.dot(v);
            const double hess = v.squaredNorm() + r.dot(a);

            if (v.squaredNorm() < 1e-12) {
                break;
            }

            double du;
            if (std::abs(hess) > 1e-8) {
                du = -grad / hess;          // 牛顿法
            } else {
                du = -grad / v.squaredNorm(); // 退化为方案二
            }

            du = std::clamp(du, -0.5, 0.5);

            const double u_next = std::clamp(u + du, u_min, u_max);

            if (std::abs(u_next - u) < 1e-6) {
                u = u_next;
                break;
            }

            u = u_next;
        }

        return u;
    }

    std::vector<double> u_table_;
    std::vector<double> s_table_;
};

#endif // __B_SPLINE_HPP_
