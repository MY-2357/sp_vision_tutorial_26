// #include "buff_solver.hpp"

// namespace auto_buff
// {
// void Buff_Solver::solvePnP(){

// }

// }  // namespace auto_buff

#include "buff_solver.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <random>
#include <cmath>

namespace auto_buff
{
    void solvePnP(cv::InputArray &imagePoints, cv::OutputArray rvec, cv::OutputArray tvec)
    {
        cv::solvePnP(buff_object_points, imagePoints, camera_matrix, distort_coeffs, rvec, tvec);
    }
    // 构造函数实现
    Buff_Solver::Buff_Solver(size_t window_size,
                             int ransac_iterations,
                             double ransac_threshold,
                             double alpha,
                             double motion_detection_threshold,
                             size_t motion_consecutive_frames)
        : window_size_(window_size),
          ransac_iterations_(ransac_iterations),
          ransac_threshold_(ransac_threshold),
          alpha_(alpha),
          motion_detection_threshold_(motion_detection_threshold),
          motion_consecutive_frames_(motion_consecutive_frames),
          motion_counter_(0),
          estimated_radius_(0.0),
          is_initialized_(false)
    {
        if (window_size < 3)
        {
            throw std::invalid_argument("window_size must be at least 3.");
        }
        if (ransac_iterations <= 0)
        {
            throw std::invalid_argument("ransac_iterations must be positive.");
        }
        if (ransac_threshold <= 0)
        {
            throw std::invalid_argument("ransac_threshold must be positive.");
        }
        if (alpha <= 0 || alpha > 1)
        {
            throw std::invalid_argument("alpha must be in (0, 1].");
        }
        if (motion_detection_threshold <= 1.0)
        {
            throw std::invalid_argument("motion_detection_threshold must be greater than 1.0.");
        }
        if (motion_consecutive_frames == 0)
        {
            throw std::invalid_argument("motion_consecutive_frames must be positive.");
        }
    }

    // 输入新点并更新估计
    bool Buff_Solver::input(const cv::Point3f &new_point)
    {
        bool was_reset = false;

        // 1. 检查是否需要重置
        if (is_initialized_ && checkAndResetIfNeeded(new_point))
        {
            std::cout << "Camera motion detected. Resetting Buff_Solver." << std::endl;
            reset();
            was_reset = true;
        }

        // 2. 将新点加入滑动窗口
        point_buffer_.push_back(new_point);
        if (point_buffer_.size() > window_size_)
        {
            point_buffer_.pop_front();
        }

        // 3. 只有当窗口内的数据点数量足够时，才进行计算
        if (point_buffer_.size() < 5)
        { // 至少需要3个点，但为了稳定性，我们要求更多
            if (!is_initialized_)
            {
                // 如果还未初始化，先用简单的平均作为初始值
                estimated_center_ = computeAverage(std::vector<cv::Point3f>(point_buffer_.begin(), point_buffer_.end()));
                estimated_radius_ = getEstimatedRadius();
                is_initialized_ = true;
            }
            return was_reset;
        }

        // 4. 使用RANSAC算法进行鲁棒的中心点估计
        cv::Point3f raw_estimate = estimateCenterWithRANSAC();

        // 5. 使用一阶低通滤波器平滑结果
        if (is_initialized_)
        {
            estimated_center_.x = alpha_ * raw_estimate.x + (1 - alpha_) * estimated_center_.x;
            estimated_center_.y = alpha_ * raw_estimate.y + (1 - alpha_) * estimated_center_.y;
            estimated_center_.z = alpha_ * raw_estimate.z + (1 - alpha_) * estimated_center_.z;
        }
        else
        {
            estimated_center_ = raw_estimate;
            is_initialized_ = true;
        }

        // 6. 更新估计的半径
        estimated_radius_ = getEstimatedRadius();

        return was_reset;
    }

    // 获取估计的中心点
    cv::Point3f Buff_Solver::getEstimatedCenter() const
    {
        return estimated_center_;
    }

    // 获取估计的半径
    double Buff_Solver::getEstimatedRadius() const
    {
        if (point_buffer_.empty() || !is_initialized_)
            return 0.0;

        double sum_dist_sq = 0.0;
        for (const auto &p : point_buffer_)
        {
            double dx = p.x - estimated_center_.x;
            double dy = p.y - estimated_center_.y;
            double dz = p.z - estimated_center_.z;
            sum_dist_sq += dx * dx + dy * dy + dz * dz;
        }
        return std::sqrt(sum_dist_sq / point_buffer_.size());
    }

    // 重置求解器
    void Buff_Solver::reset()
    {
        point_buffer_.clear();
        motion_counter_ = 0;
        is_initialized_ = false;
        estimated_center_ = cv::Point3f(0, 0, 0);
        estimated_radius_ = 0.0;
    }

    // 检查是否已初始化
    bool Buff_Solver::isInitialized() const
    {
        return is_initialized_;
    }

    // --- 私有函数实现 ---

    cv::Point3f Buff_Solver::computeAverage(const std::vector<cv::Point3f> &points) const
    {
        cv::Point3f avg(0, 0, 0);
        for (const auto &p : points)
        {
            avg += p;
        }
        avg.x /= points.size();
        avg.y /= points.size();
        avg.z /= points.size();
        return avg;
    }

    cv::Point3f Buff_Solver::estimateCenterWithRANSAC() const
    {
        if (point_buffer_.size() < 3)
        {
            throw std::runtime_error("Not enough points to perform RANSAC.");
        }

        // 初始化随机数生成器
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<size_t> dist(0, point_buffer_.size() - 1);

        int best_inlier_count = 0;
        std::vector<cv::Point3f> best_inliers;

        // RANSAC迭代
        for (int i = 0; i < ransac_iterations_; ++i)
        {
            // 1. 随机采样3个点 (最小子集)
            std::vector<cv::Point3f> sample;
            while (sample.size() < 3)
            {
                size_t idx = dist(gen);
                // 避免重复采样同一个点
                bool is_duplicate = false;
                for (const auto &p : sample)
                {
                    if (cv::norm(p - point_buffer_[idx]) < 1e-6)
                    {
                        is_duplicate = true;
                        break;
                    }
                }
                if (!is_duplicate)
                {
                    sample.push_back(point_buffer_[idx]);
                }
            }

            // 2. 计算这3个点的“模型”，即它们的圆心 (对于3个点，圆心是这3个点的平均)
            cv::Point3f model_center = computeAverage(sample);

            // 3. 计算所有点到该模型的距离，统计内点数量
            std::vector<cv::Point3f> current_inliers;
            for (const auto &p : point_buffer_)
            {
                double dist = cv::norm(p - model_center);
                if (dist < ransac_threshold_)
                {
                    current_inliers.push_back(p);
                }
            }

            // 4. 如果当前内点集更大，则更新最佳模型
            if (current_inliers.size() > best_inlier_count)
            {
                best_inlier_count = current_inliers.size();
                best_inliers = current_inliers;
            }
        }

        // 如果找到了足够的内点，则用所有内点的平均作为最终估计
        if (best_inliers.size() > point_buffer_.size() / 2)
        { // 要求内点至少占一半
            return computeAverage(best_inliers);
        }
        else
        {
            // 如果RANSAC失败（可能所有点都很分散），则退回到简单的平均值
            // std::cerr << "Warning: RANSAC found insufficient inliers. Reverting to simple average." << std::endl;
            return computeAverage(std::vector<cv::Point3f>(point_buffer_.begin(), point_buffer_.end()));
        }
    }

    bool Buff_Solver::checkAndResetIfNeeded(const cv::Point3f &new_point)
    {
        // 计算新点到当前估计中心的距离
        double distance_to_center = cv::norm(new_point - estimated_center_);

        // 动态阈值 = K * 当前估计的半径
        double threshold = motion_detection_threshold_ * estimated_radius_;

        // 如果距离远大于半径，则认为可能发生了相机运动
        if (distance_to_center > threshold)
        {
            motion_counter_++;
            // 如果连续N帧都超出阈值，则确认发生运动
            if (motion_counter_ >= motion_consecutive_frames_)
            {
                return true;
            }
        }
        else
        {
            // 否则，重置计数器
            motion_counter_ = 0;
        }

        return false;
    }

} // namespace auto_buff
