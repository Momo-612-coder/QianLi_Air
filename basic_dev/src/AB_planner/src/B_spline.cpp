#include "B_spline.hpp"

// ------------------ B样条类的实现 -----------------

BSpline::BSpline()
{
    
}

BSpline::~BSpline()
{
}

// 从A_star的路径点列表中提取控制点，构建B样条曲线
void BSpline::setControlPoints(const std::vector<Eigen::Vector3d>& control_points)
{
    control_points_ = control_points;
}

int BSpline::numSegments() const
{
    // 计算B样条曲线的分段数量，分段数量等于控制点数量减去阶数
    if (control_points_.size() < degree_ + 1) {
        ROS_WARN("Not enough control points to form a B-spline curve. Need at least %d control points for degree %d.", degree_ + 1, degree_);
        return 0; // 如果控制点数量不足以构成一个完整的B样条曲线，则返回0
    }
    return control_points_.size() - degree_;
}

double BSpline::maxU() const
{
    // 计算B样条曲线的最大参数值，最大参数值等于分段数量乘以节点间隔
    return numSegments() * knot_interval_;
}

// 将 u 规范化为段索引 k 和局部参数 dt ∈ [0,1), 其中 k 是 B 样条曲线的段索引，dt 是该段内的局部参数
std::pair<int, double> BSpline::normalizeU(double u) const
{
    double t = u / knot_interval_; // 将全局参数 u 转换为以 knot_interval_ 为单位的参数 t
    int k = static_cast<int>(std::floor(t)); // 计算段索引 k，取整向下
    k = std::min(std::max(k, 0), numSegments() - 1); // 将段索引 k 限制在有效范围内 [0, numSegments() - 1]
    double dt = t - k; // 计算局部参数 dt，等于 t 减去段索引 k
    return {k, dt};
}

// 计算B样条曲线的基函数值，输入局部参数 dt，输出4阶B样条的4个基函数值
Eigen::RowVector4d BSpline::basisFunctions(double dt)
{
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    Eigen::RowVector4d N;
    N(0) = (1 - 3 * dt + 3 * dt2 - dt3) / 6.0; // N0,3(dt)
    N(1) = (3 * dt3 - 6 * dt2 + 4) / 6.0; // N1,3(dt)
    N(2) = (-3 * dt3 + 3 * dt2 + 3 * dt + 1) / 6.0; // N2,3(dt)
    N(3) = dt3 / 6.0; // N3,3(dt)
    return N;
}

// 计算B样条曲线的导数基函数值，输入局部参数 dt 和导数阶数 m，输出4阶B样条的4个导数基函数值, m = 1 表示一阶导数，m = 2 表示二阶导数
Eigen::RowVector4d BSpline::derivativeBasis(double dt, int m)
{
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    Eigen::RowVector4d dN;
    if (m == 1) {
        dN(0) = (-3 + 6 * dt - 3 * dt2) / 6.0; // N0,3'(dt)
        dN(1) = (9 * dt2 - 12 * dt) / 6.0; // N1,3'(dt)
        dN(2) = (-9 * dt2 + 6 * dt + 3) / 6.0; // N2,3'(dt)
        dN(3) = (3 * dt2) / 6.0; // N3,3'(dt)
    } else if (m == 2) {
        dN(0) = (6 - 6 * dt) / 6.0; // N0,3''(dt)
        dN(1) = (18 * dt - 12) / 6.0; // N1,3''(dt)
        dN(2) = (-18 * dt + 6) / 6.0; // N2,3''(dt)
        dN(3) = (6 * dt) / 6.0; // N3,3''(dt)
    } else {
        ROS_WARN("Unsupported derivative order m=%d. Only m=1 and m=2 are supported.", m);
        dN.setZero(); // 如果导数阶数不支持，则返回全零的导数基函数值
    }
    return dN;
}

// 获取第 k 段的4个控制点，输入段索引 k，输出一个 4x3 的矩阵，每行对应一个控制点的 (x, y, z) 坐标
Eigen::Matrix<double, 4, 3> BSpline::segmentCtrlPoints(int k) const
{
    Eigen::Matrix<double, 4, 3> ctrl_pts;
    for (int i = 0; i < 4; ++i) {
        ctrl_pts.row(i) = control_points_[k + i].transpose(); // 将控制点的 (x, y, z) 坐标存储在矩阵的行中
    }
    return ctrl_pts;
}

// 计算B样条曲线在参数 u 处的位置，输入全局参数 u，输出曲线在该参数处的 (x, y, z) 坐标
Eigen::Vector3d BSpline::position(double u) const
{
    const auto normalized = normalizeU(u); // 将全局参数 u 规范化为段索引 k 和局部参数 dt
    const int k = normalized.first;
    const double dt = normalized.second;
    Eigen::RowVector4d N = basisFunctions(dt); // 计算局部参数 dt 的基函数值
    Eigen::Matrix<double, 4, 3> ctrl_pts = segmentCtrlPoints(k); // 获取第 k 段的4个控制点
    return N * ctrl_pts; // 计算曲线位置，基函数值与控制点坐标的乘积
}

// 计算B样条曲线在参数 u 处的速度，输入全局参数 u，输出曲线在该参数处的速度向量 (vx, vy, vz)
Eigen::Vector3d BSpline::velocity(double u) const
{
    const auto normalized = normalizeU(u); // 将全局参数 u 规范化为段索引 k 和局部参数 dt
    const int k = normalized.first;
    const double dt = normalized.second;
    Eigen::RowVector4d dN = derivativeBasis(dt, 1); // 计算局部参数 dt 的一阶导数基函数值
    Eigen::Matrix<double, 4, 3> ctrl_pts = segmentCtrlPoints(k); // 获取第 k 段的4个控制点
    return (dN * ctrl_pts) / knot_interval_; // 计算曲线速度，导数基函数值与控制点坐标的乘积除以节点间隔
}

// 计算B样条曲线在参数 u 处的加速度，输入全局参数 u，输出曲线在该参数处的加速度向量 (ax, ay, az)
Eigen::Vector3d BSpline::acceleration(double u) const
{
    const auto normalized = normalizeU(u); // 将全局参数 u 规范化为段索引 k 和局部参数 dt
    const int k = normalized.first;
    const double dt = normalized.second;
    Eigen::RowVector4d d2N = derivativeBasis(dt, 2); // 计算局部参数 dt 的二阶导数基函数值
    Eigen::Matrix<double, 4, 3> ctrl_pts = segmentCtrlPoints(k); // 获取第 k 段的4个控制点
    return (d2N * ctrl_pts) / (knot_interval_ * knot_interval_); // 计算曲线加速度，二阶导数基函数值与控制点坐标的乘积除以节点间隔的平方
}

// -------------------------------- ArcLengthTable 类的实现 ------------------ --------------
// 构建弧长表，输入B样条曲线对象、参数范围和采样点数量，输出弧长表
void ArcLengthTable::build(const BSpline& bspline, double u_start, double u_end, int num_points)
{
    u_table_.clear();
    s_table_.clear();

    if (num_points < 2 || u_end <= u_start) {
        ROS_WARN("Invalid parameters for building arc length table. num_points=%d, u_start=%.2f, u_end=%.2f",
                 num_points, u_start, u_end);
        return;
    }

    u_table_.resize(num_points);
    s_table_.resize(num_points);

    const double du = (u_end - u_start) / (num_points - 1);
    double s = 0.0;

    u_table_[0] = u_start;
    s_table_[0] = 0.0;

    constexpr double t = 0.577350269189626; // 1 / sqrt(3)

    for (int i = 1; i < num_points; ++i) {
        const double u_prev = u_table_[i - 1];
        const double u_cur = u_start + i * du;

        const double mid = 0.5 * (u_prev + u_cur);
        const double half = 0.5 * (u_cur - u_prev);

        const double v1 = bspline.velocity(mid - half * t).norm();
        const double v2 = bspline.velocity(mid + half * t).norm();

        s += (v1 + v2) * half;

        u_table_[i] = u_cur;
        s_table_[i] = s;
    }
}

// 根据参数 u 获取对应的弧长 s，输入全局参数 u，输出曲线从起点到参数 u 处的弧长
double ArcLengthTable::getLength(double u) const
{
    if (u_table_.empty() || s_table_.empty() || u_table_.size() != s_table_.size()) {
        return 0.0;
    }
    if (u <= u_table_.front()) {
        return s_table_.front();
    }
    if (u >= u_table_.back()) {
        return s_table_.back();
    }

    auto upper = std::lower_bound(u_table_.begin(), u_table_.end(), u);
    const size_t idx = static_cast<size_t>(std::distance(u_table_.begin(), upper));

    const double u0 = u_table_[idx - 1];
    const double u1 = u_table_[idx];
    const double s0 = s_table_[idx - 1];
    const double s1 = s_table_[idx];

    const double alpha = (u - u0) / std::max(u1 - u0, 1e-6);

    return s0 + alpha * (s1 - s0);
}

// 根据弧长 s 获取对应的参数 u，输入曲线从起点到某点的弧长 s，输出该点对应的全局参数 u
double ArcLengthTable::getUfromS(double s) const
{
    if (u_table_.empty() || s_table_.empty() || u_table_.size() != s_table_.size()) {
        return 0.0;
    }
    if (s <= s_table_.front()) {
        return u_table_.front();
    }
    if (s >= s_table_.back()) {
        return u_table_.back();
    }
    auto upper = std::lower_bound(s_table_.begin(), s_table_.end(), s);
    const size_t idx = static_cast<size_t>(std::distance(s_table_.begin(), upper));
    const double s0 = s_table_[idx - 1];
    const double s1 = s_table_[idx];
    const double u0 = u_table_[idx - 1];
    const double u1 = u_table_[idx];
    const double alpha = (s - s0) / std::max(s1 - s0, 1e-6);
    return u0 + alpha * (u1 - u0);
}

// 根据参数 u 获取对应的弧长 s，输入全局参数 u，输出曲线从起点到参数 u 处的弧长
double ArcLengthTable::getSfromU(double u) const
{
    if (u_table_.empty() || s_table_.empty() || u_table_.size() != s_table_.size()) {
        return 0.0;
    }
    if (u <= u_table_.front()) {
        return s_table_.front();
    }
    if (u >= u_table_.back()) {
        return s_table_.back();
    }

    auto upper = std::lower_bound(u_table_.begin(), u_table_.end(), u);
    const size_t idx = static_cast<size_t>(std::distance(u_table_.begin(), upper));

    const double u0 = u_table_[idx - 1];
    const double u1 = u_table_[idx];
    const double s0 = s_table_[idx - 1];
    const double s1 = s_table_[idx];

    const double alpha = (u - u0) / std::max(u1 - u0, 1e-6);

    return s0 + alpha * (s1 - s0);
}
