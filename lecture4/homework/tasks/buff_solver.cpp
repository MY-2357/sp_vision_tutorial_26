#include "buff_solver.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

namespace auto_buff
{
    Buff_Solver::Buff_Solver(int max_points)
        : max_points_(max_points)
    {
    }

    void solvePnP(cv::InputArray &imagePoints, cv::OutputArray rvec, cv::OutputArray tvec)
    {
        cv::solvePnP(buff_object_points, imagePoints, camera_matrix, distort_coeffs, rvec, tvec);
    }


} // namespace auto_buff
