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

// 六个关键点的坐标（以旋转中心为坐标原点，并认为扇叶垂直向上）
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
static const auto point_1 = cv::Point3f(0.0,700.0+382.0/2,0.0);
static const auto point_2 = cv::Point3f(-320.0 / 2, 700.0/2, 0.0);
static const auto point_3 = cv::Point3f(0.0, 700.0-382.0/2, 0.0);
static const auto point_4 = cv::Point3f(320.0 / 2, 700.0/2, 0.0);
static const auto point_5 = cv::Point3f(0.0, 700.0/2, 0.0);
static const auto point_6 = cv::Point3f(0.0, 700.0/2-382.0/2-330.0, 0.0);

// static const auto point_1 = cv::Point3f(0.0, 330.0 + 382.0 - ADJUST_Y, 0.0);
// static const auto point_2 = cv::Point3f(-320.0 / 2, 330.0 + 382.0 / 2 - ADJUST_Y, 0.0);
// static const auto point_3 = cv::Point3f(0.0, 330.0 - ADJUST_Y, 0.0);
// static const auto point_4 = cv::Point3f(320.0 / 2, 330.0 + 382.0 / 2 - ADJUST_Y, 0.0);
// static const auto point_5 = cv::Point3f(0.0, 330.0 + 382.0 / 2 - ADJUST_Y, 0.0);
// static const auto point_6 = cv::Point3f(0.0, 0.0 - ADJUST_Y, 0.0);


static const std::vector<cv::Point3f> buff_object_points =
    {point_1, point_2, point_3, point_4, point_5, point_6};

namespace auto_buff
{

    void solvePnP(cv::InputArray &imagePoints, cv::OutputArray rvec, cv::OutputArray tvec);
    class Buff_Solver
    {
    public:
        Buff_Solver(int max_points = 30);


        cv::Point3f updateAndSolve(const cv::Point3f &symbol_center);

    private:
        bool fitCircle3D(const std::vector<cv::Point3f> &points, cv::Point3f &center, float &radius);

        int max_points_;
        std::vector<cv::Point3f> history_points;

        // 相机内参（需外部赋值）
        cv::Mat camera_matrix;
        cv::Mat distort_coeffs;

        // 能量机关几何参数（3D 模型点）
        std::vector<cv::Point3f> buff_object_points;
    };

} // namespace auto_buff

#endif // AUTO_BUFF__SOLVER_HPP
