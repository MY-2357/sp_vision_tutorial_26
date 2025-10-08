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

    // 更新历史点并求旋转中心
    cv::Point3f Buff_Solver::updateAndSolve(const cv::Point3f &symbol_center)
    {
        history_points.push_back(symbol_center);
        if ((int)history_points.size() > max_points_)
            history_points.erase(history_points.begin());

        cv::Point3f rotation_center;
        float radius;
        if (fitCircle3D(history_points, rotation_center, radius))
            return rotation_center;
        else
            return symbol_center; // 拟合失败返回当前符中心
    }

    // 简单的3D圆拟合（投影到最优平面再拟合圆）
    bool Buff_Solver::fitCircle3D(const std::vector<cv::Point3f> &points, cv::Point3f &center, float &radius)
    {
        if (points.size() < 3)
            return false;

        // 1. PCA 拟合平面
        cv::Mat data_pts(points.size(), 3, CV_64F);
        for (size_t i = 0; i < points.size(); ++i)
        {
            data_pts.at<double>(i, 0) = points[i].x;
            data_pts.at<double>(i, 1) = points[i].y;
            data_pts.at<double>(i, 2) = points[i].z;
        }

        cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);
        cv::Vec3d plane_normal = pca_analysis.eigenvectors.row(2); // 法向
        cv::Point3d plane_center = cv::Point3d(
            pca_analysis.mean.at<double>(0),
            pca_analysis.mean.at<double>(1),
            pca_analysis.mean.at<double>(2));

        // 2. 构建平面局部坐标系
        cv::Vec3d u = pca_analysis.eigenvectors.row(0); // 最大方差方向
        cv::Vec3d v = pca_analysis.eigenvectors.row(1); // 第二大方差方向

        // 3. 投影历史点到平面坐标系 (u,v)
        std::vector<cv::Point2f> pts2d;
        for (const auto &pt : points)
        {
            cv::Vec3d vec(pt.x - plane_center.x, pt.y - plane_center.y, pt.z - plane_center.z);
            double x = vec.dot(u);
            double y = vec.dot(v);
            pts2d.push_back(cv::Point2f(x, y));
        }

        // 4. 最小二乘拟合圆
        int N = pts2d.size();
        cv::Mat A(N, 3, CV_64F);
        cv::Mat B(N, 1, CV_64F);

        for (int i = 0; i < N; ++i)
        {
            double x = pts2d[i].x;
            double y = pts2d[i].y;
            A.at<double>(i, 0) = 2.0 * x;
            A.at<double>(i, 1) = 2.0 * y;
            A.at<double>(i, 2) = 1.0;
            B.at<double>(i, 0) = x * x + y * y;
        }

        cv::Mat sol;
        cv::solve(A, B, sol, cv::DECOMP_SVD);

        double cx = sol.at<double>(0, 0);
        double cy = sol.at<double>(1, 0);
        radius = std::sqrt(sol.at<double>(2, 0) + cx * cx + cy * cy);

        // 5. 映射回 3D 坐标系
        cv::Vec3d plane_center_vec(plane_center.x, plane_center.y, plane_center.z);
        cv::Vec3d center_vec = plane_center_vec + cx * u + cy * v;
        center = cv::Point3f(center_vec[0], center_vec[1], center_vec[2]);

        return true;
    }

} // namespace auto_buff
