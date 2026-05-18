#ifndef TRAJECTORY_PLANNER__CUBIC_SPLINE_HPP_
#define TRAJECTORY_PLANNER__CUBIC_SPLINE_HPP_

#include <vector>
#include <stdexcept>
#include <cmath>

namespace trajectory_planner
{

// 一维三次样条插值类
class CubicSpline
{
public:
    CubicSpline() = default;

    // 根据数据点 x 和 y 初始化样条曲线
    bool init(const std::vector<double>& x, const std::vector<double>& y)
    {
        if (x.size() != y.size() || x.size() < 3) {
            return false;
        }

        x_ = x;
        y_ = y;
        int n = x.size() - 1;

        std::vector<double> h(n);
        for (int i = 0; i < n; ++i) {
            h[i] = x[i + 1] - x[i];
            if (h[i] <= 0.0) {
                return false; // x 必须严格单调递增
            }
        }

        std::vector<double> alpha(n);
        for (int i = 1; i < n; ++i) {
            alpha[i] = 3.0 / h[i] * (y[i + 1] - y[i]) - 3.0 / h[i - 1] * (y[i] - y[i - 1]);
        }

        std::vector<double> l(n + 1);
        std::vector<double> mu(n + 1);
        std::vector<double> z(n + 1);
        
        l[0] = 1.0;
        mu[0] = 0.0;
        z[0] = 0.0;

        for (int i = 1; i < n; ++i) {
            l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[n] = 1.0;
        z[n] = 0.0;
        
        c_.resize(n + 1);
        b_.resize(n);
        d_.resize(n);

        c_[n] = 0.0;
        for (int j = n - 1; j >= 0; --j) {
            c_[j] = z[j] - mu[j] * c_[j + 1];
            b_[j] = (y[j + 1] - y[j]) / h[j] - h[j] * (c_[j + 1] + 2.0 * c_[j]) / 3.0;
            d_[j] = (c_[j + 1] - c_[j]) / (3.0 * h[j]);
        }

        return true;
    }

    // 计算给定 x 处的值 (位置)
    double calc(double t) const
    {
        if (x_.empty()) return 0.0;
        if (t <= x_.front()) return y_.front();
        if (t >= x_.back()) return y_.back();

        int i = search_index(t);
        double dx = t - x_[i];
        return y_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
    }

    // 计算给定 x 处的一阶导数 (速度)
    double calc_d(double t) const
    {
        if (x_.empty() || t < x_.front() || t > x_.back()) return 0.0;

        int i = search_index(t);
        double dx = t - x_[i];
        return b_[i] + 2.0 * c_[i] * dx + 3.0 * d_[i] * dx * dx;
    }

    // 计算给定 x 处的二阶导数 (加速度)
    double calc_dd(double t) const
    {
        if (x_.empty() || t < x_.front() || t > x_.back()) return 0.0;

        int i = search_index(t);
        double dx = t - x_[i];
        return 2.0 * c_[i] + 6.0 * d_[i] * dx;
    }

private:
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> b_;
    std::vector<double> c_;
    std::vector<double> d_;

    int search_index(double t) const
    {
        // 简单的二分查找
        int left = 0;
        int right = x_.size() - 1;
        while (left < right) {
            int mid = left + (right - left) / 2;
            if (x_[mid] <= t && t < x_[mid + 1]) {
                return mid;
            }
            if (x_[mid] < t) {
                left = mid + 1;
            } else {
                right = mid;
            }
        }
        return left;
    }
};

// 二维样条曲线，用于同时插值 X 和 Y 坐标
class Spline2D
{
public:
    Spline2D() = default;

    bool init(const std::vector<double>& x, const std::vector<double>& y)
    {
        if (x.size() != y.size() || x.size() < 3) return false;

        s_.clear();
        s_.push_back(0.0);
        double dx, dy;
        for (size_t i = 1; i < x.size(); ++i) {
            dx = x[i] - x[i - 1];
            dy = y[i] - y[i - 1];
            s_.push_back(s_.back() + std::hypot(dx, dy));
        }

        bool sx_ok = sx_.init(s_, x);
        bool sy_ok = sy_.init(s_, y);

        return sx_ok && sy_ok;
    }

    // 根据累积弧长 s 计算 X 和 Y
    void calc_position(double s, double& x, double& y) const
    {
        x = sx_.calc(s);
        y = sy_.calc(s);
    }

    // 根据累积弧长 s 计算航向角 yaw
    double calc_yaw(double s) const
    {
        double dx = sx_.calc_d(s);
        double dy = sy_.calc_d(s);
        return std::atan2(dy, dx);
    }

    double get_total_length() const
    {
        if (s_.empty()) return 0.0;
        return s_.back();
    }

private:
    std::vector<double> s_; // 累积弧长
    CubicSpline sx_;
    CubicSpline sy_;
};

} // namespace trajectory_planner

#endif // TRAJECTORY_PLANNER__CUBIC_SPLINE_HPP_