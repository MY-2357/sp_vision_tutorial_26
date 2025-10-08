// #ifndef AUTO_BUFF__SOLVER_HPP
// #define AUTO_BUFF__SOLVER_HPP

// namespace auto_buff
// {
// class Buff_Solver
// {
// public:
//     void solvePnP();
// };
// }  // namespace auto_buff
// #endif  // SOLVER_HPP

#ifndef AUTO_BUFF__SOLVER_HPP
#define AUTO_BUFF__SOLVER_HPP

#include <opencv2/opencv.hpp>
#include <vector>

// 下面的参数直接从培训时用的main.cpp中复制过来

// clang-format off
//  相机内参
static const cv::Mat camera_matrix =
    (cv::Mat_<double>(3, 3) <<  1286.307063384126 , 0                  , 645.34450819155256, 
                                0                 , 1288.1400736562441 , 483.6163720308021 , 
                                0                 , 0                  , 1                   );
// 畸变系数
static const cv::Mat distort_coeffs =
    (cv::Mat_<double>(1, 5) << -0.47562935060124745, 0.21831745829617311, 0.0004957613589406044, -0.00034617769548693592, 0);
// clang-format on

// 六个关键点的坐标（由于之后要反推出旋转中心，但是现在还没有旋转中心的坐标，故以扇叶的头部中心为坐标原点，并认为扇叶垂直向上）
/*
           ^ y
           |
           |
-----------+-------->x
           |
           |
单位:mm

*/
#define ADJUST_Y (330.0 + 382.0 / 2)
// #define ADJUST_Y 0.0

static const auto point_1 = cv::Point3f(0.0, 330.0 + 382.0 - ADJUST_Y, 0.0);
static const auto point_2 = cv::Point3f(-320.0 / 2, 330.0 + 382.0 / 2 - ADJUST_Y, 0.0);
static const auto point_3 = cv::Point3f(0.0, 330.0 - ADJUST_Y, 0.0);
static const auto point_4 = cv::Point3f(320.0 / 2, 330.0 + 382.0 / 2 - ADJUST_Y, 0.0);
static const auto point_5 = cv::Point3f(0.0, 330.0 + 382.0 / 2 - ADJUST_Y, 0.0);
static const auto point_6 = cv::Point3f(0.0, 0.0 - ADJUST_Y, 0.0);

static const std::vector<cv::Point3f> buff_object_points =
    {point_1, point_2, point_3, point_4, point_5, point_6};

namespace auto_buff
{

    void solvePnP(cv::InputArray &imagePoints, cv::OutputArray rvec, cv::OutputArray tvec);
    class Buff_Solver
    {
    public:
        /**
         * @brief 构造函数
         * @param window_size 用于计算的滑动窗口大小。
         * @param ransac_iterations RANSAC算法的迭代次数。
         * @param ransac_threshold RANSAC算法中，判断一个点是否为内点的距离阈值（单位：米）。
         * @param alpha 一阶低通滤波器的平滑系数 (0 < alpha <= 1)。
         * @param motion_detection_threshold 相机运动检测的动态阈值系数。当新点到估计中心的距离超过此系数乘以估计半径时，认为可能发生运动。
         * @param motion_consecutive_frames 连续多少帧满足运动条件时，才判定为相机发生运动。
         */
        Buff_Solver(size_t window_size = 100,
                    int ransac_iterations = 100,
                    double ransac_threshold = 0.02, // 2厘米
                    double alpha = 0.3,
                    double motion_detection_threshold = 2.5,
                    size_t motion_consecutive_frames = 5);

        /**
         * @brief 输入一个新的三维点，并更新中心点的估计。
         * @param new_point 新观测到的三维点坐标。
         * @return 如果求解器因检测到相机运动而重置，则返回 true；否则返回 false。
         */
        bool input(const cv::Point3f &new_point);

        /**
         * @brief 获取当前估计的中心点。
         * @return 如果已经有足够的数据进行估计，则返回估计的中心点；否则返回一个默认的Point3f。
         */
        cv::Point3f getEstimatedCenter() const;

        /**
         * @brief 获取滑动窗口中所有点到当前估计中心点的平均距离（即估计的旋转半径）。
         * @return 估计的旋转半径。
         */
        double getEstimatedRadius() const;

        /**
         * @brief 清空所有缓存数据和状态。
         */
        void reset();

        /**
         * @brief 检查求解器是否已初始化（即是否有足够的数据进行过一次有效的估计）。
         * @return 如果已初始化，则返回 true；否则返回 false。
         */
        bool isInitialized() const;

    private:
        /**
         * @brief 计算一组点的算术平均值。
         * @param points 输入的点集。
         * @return 点集的平均点。
         */
        cv::Point3f computeAverage(const std::vector<cv::Point3f> &points) const;

        /**
         * @brief 使用RANSAC算法鲁棒地估计中心点。
         * @return 估计出的中心点。
         */
        cv::Point3f estimateCenterWithRANSAC() const;

        /**
         * @brief 检查是否应该重置求解器（即检测相机是否发生了转动）。
         * @param new_point 新输入的点。
         * @return 如果应该重置，则返回 true；否则返回 false。
         */
        bool checkAndResetIfNeeded(const cv::Point3f &new_point);

        // --- 成员变量 ---
        size_t window_size_;      // 滑动窗口大小
        int ransac_iterations_;   // RANSAC迭代次数
        double ransac_threshold_; // RANSAC内点距离阈值
        double alpha_;            // 低通滤波器平滑系数

        // 相机运动检测参数
        double motion_detection_threshold_; // 动态阈值系数
        size_t motion_consecutive_frames_;  // 连续帧数阈值
        size_t motion_counter_;             // 当前连续超出阈值的帧数

        std::deque<cv::Point3f> point_buffer_; // 存储最近的点
        cv::Point3f estimated_center_;         // 当前估计的中心点
        double estimated_radius_;              // 当前估计的半径
        bool is_initialized_;                  // 状态是否已初始化
    };

} // namespace auto_buff

#endif // AUTO_BUFF__SOLVER_HPP
