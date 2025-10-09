/* 修改说明：
1、更改坐标系x,y轴方向
2、取消使用最小二乘法求解PnP，改为利用tvec和rvec直接解算
3、当前状态：未完成
*/

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
           
           |
           |
-----------+-------->x
           |
           |
           y
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


} // namespace auto_buff

#endif // AUTO_BUFF__SOLVER_HPP
