#include "buff_solver.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

namespace auto_buff
{

    void solvePnP(cv::InputArray &imagePoints, cv::OutputArray rvec, cv::OutputArray tvec)
    {
        cv::solvePnP(buff_object_points, imagePoints, camera_matrix, distort_coeffs, rvec, tvec);
    }

    void calculateRotationCenter(cv::Mat &rvec, cv::Mat &tvec, cv::Point3f &point)
    {
        // 直接利用当前的位置与位姿寻找中心点(忽略z, 只考虑x,y)
        auto inputPos = cv::Point3f(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        // double yaw = atan2(rmat.at<double>(0, 2), rmat.at<double>(2, 2));
        // double pitch = -asin(rmat.at<double>(1, 2));
        // 我们只考虑roll,不计算别的。由于开始是设置为扇叶垂直向上，因此roll的处理被稍微复杂化了，顺时针为正，逆时针为负，极轴在y上。如图:
        /*
                ^
            -   |   +
                |
                0
        */
        // 取值:[-pi,pi]，开闭未知
        // inputPos单位为mm,旋转中心到符中心为700.0mm
        double roll = atan2(rmat.at<double>(1, 0), rmat.at<double>(1, 1));
        const double radius = 700.0;

        // 计算旋转中心点(其实这里也不太清楚怎么计算，坐标系搞不懂，不过就2*2四种情况，总有一个是对的)
        double x = inputPos.x - radius * sin(roll);
        double y = inputPos.y + radius * cos(roll);

        // 不行挨个试
        /*

        double x = inputPos.x - radius * cos(roll);
        double y = inputPos.y + radius * sin(roll);

        double x = inputPos.x + radius * sin(roll);
        double y = inputPos.y - radius * cos(roll);

        double x = inputPos.x + radius * cos(roll);
        double y = inputPos.y - radius * sin(roll);

        */

        point = cv::Point3f(x, y, 0.0);

        // // 下面这一段代码是gpt写的，不知道对不对
        // //  将rvec转为旋转矩阵
        // cv::Mat rmat;
        // cv::Rodrigues(rvec, rmat);

        // const double radius = 700.0;
        // // 扇叶到旋转中心的向量，在扇叶局部坐标系中固定
        // cv::Mat offset = (cv::Mat_<double>(3, 1) << 0.0, radius, 0.0);

        // // 将局部向量旋转到相机坐标系，再加上扇叶中心坐标
        // cv::Mat center_vec = rmat * offset + tvec;

        // point = cv::Point3f(center_vec.at<double>(0),
        //                     center_vec.at<double>(1),
        //                     center_vec.at<double>(2));
    }

} // namespace auto_buff
