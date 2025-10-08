#include "tasks/buff_detector.hpp"
#include "io/camera.hpp"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include "tools/plotter.hpp"
#include "tasks/buff_solver.hpp"

int main()
{

    // 下面的代码调试用
    cv::VideoCapture cap("assets/test.avi");

    if (!cap.isOpened())
    {
        std::cerr << "无法打开视频文件!" << std::endl;
        return -1;
    }

    /* io::Camera camera(2.5, 16.9, "2bdf:0001");*/
    std::chrono::steady_clock::time_point timestamp;
    auto_buff::Buff_Detector detector;
    tools::Plotter plotter;
    // 自定义参数创建求解器实例
    size_t window_size = 120;        // 使用最近的120个点进行计算
    int ransac_iterations = 150;     // RANSAC迭代150次
    double ransac_threshold = 25; // RANSAC内点阈值为2.5厘米
    double alpha = 0.25;             // 平滑系数
    double motion_threshold = 2.2;   // 相机运动检测阈值（半径的2.2倍）
    size_t motion_frames = 4;        // 连续4帧超过阈值则判定为运动

    auto_buff::Buff_Solver solver(
        window_size,
        ransac_iterations,
        ransac_threshold,
        alpha,
        motion_threshold,
        motion_frames);

    cv::Mat current_rvec, current_tvec; // 新增：存储当前扇叶的位姿

    while (true)
    {
        cv::Mat img;
        // 下面的代码调试用
        cap >> img;

        if (img.empty())
        {
            std::cout << "视频播放完毕或无法读取帧" << std::endl;
            break;
        }

        auto fanblades = detector.detect(img);

        /*
        camera.read(img, timestamp);
        auto fanblades = detector.detect(img);
        */

        // plotjuggler的data的定义提前
        nlohmann::json data;

        // 当前扇叶编号的累计计数
        int count = 0;
        // 解算后的扇叶中心坐标(为了简单，我们只用第一个扇叶的位姿)
        cv::Point3f real_fanblade_c_point;

        cv::Mat display_img = img.clone();
        for (const auto &fanblade : fanblades)
        {
            cv::Scalar color;
            std::string type_name;
            switch (fanblade.type)
            {
            case auto_buff::_target:
                color = cv::Scalar(0, 255, 0);
                type_name = "_target";
                break;
            case auto_buff::_light:
                color = cv::Scalar(0, 255, 255);
                type_name = "_light";
                break;
            case auto_buff::_unlight:
                color = cv::Scalar(0, 0, 255);
                type_name = "_unlight";
                break;
            }

            std::vector<cv::Point2f> imagePoints;

            // 绘制并记录关键点
            for (size_t i = 0; i < fanblade.points.size(); ++i)
            {
                cv::circle(display_img, fanblade.points[i], 3, color, -1);
                cv::putText(display_img, std::to_string(i),
                            cv::Point(fanblade.points[i].x + 5, fanblade.points[i].y - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
                // 向imagePoints中添加关键点
                imagePoints.push_back(fanblade.points[i]);
            }

            // 绘制中心点
            cv::circle(display_img, fanblade.center, 5, color, -1);
            cv::putText(display_img, "CENTER",
                        cv::Point(fanblade.center.x + 10, fanblade.center.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);

            // 绘制类型标签
            cv::putText(display_img, type_name,
                        cv::Point(fanblade.center.x - 20, fanblade.center.y - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);

            // 解算当前旋转扇叶的位姿
            cv::Mat rvec, tvec;

            auto_buff::solvePnP(imagePoints, rvec, tvec);

            // 添加每一个扇叶中心的数据
            if (fanblades.size())
            {
                std::string fanblade_center_x_index = "fanblade_" + std::to_string(count) + "_c_x";
                std::string fanblade_center_y_index = "fanblade_" + std::to_string(count) + "_c_y";
                std::string fanblade_center_z_index = "fanblade_" + std::to_string(count) + "_c_z";

                // 单位转换：mm → m（关键修改1）
                double cx_m = tvec.at<double>(0) / 1000.0;
                double cy_m = tvec.at<double>(1) / 1000.0;
                double cz_m = tvec.at<double>(2) / 1000.0;

                data[fanblade_center_x_index] = cx_m;
                data[fanblade_center_y_index] = cy_m;
                data[fanblade_center_z_index] = cz_m;

                if (count == 0)
                {
                    real_fanblade_c_point.x = cx_m; // 米单位
                    real_fanblade_c_point.y = cy_m;
                    real_fanblade_c_point.z = cz_m;
                    // 保存当前扇叶的rvec和tvec，用于后续投影（关键修改3）
                    current_rvec = rvec.clone();
                    current_tvec = tvec.clone();
                }
            }

            count++;
        }

        plotter.plot(data);

        // 下面是gpt对于计算旋转中心的代码
        // if (!fanblades.empty())
        // {
        //     // 符中心
        //     cv::Point2f symbol_center = real_fanblade_c_point;

        //     // 调用 solver 得到旋转中心
        //     cv::Point2f rotation_center = solver.updateAndSolve(symbol_center);

        //     // 绘制
        //     cv::circle(display_img, rotation_center, 8, cv::Scalar(255, 255, 0), -1);
        //     cv::putText(display_img, "CENTER",
        //                 cv::Point(rotation_center.x + 10, rotation_center.y - 10),
        //                 cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

        //     // PlotJuggler 输出
        //     nlohmann::json data;
        //     data["symbol_center_x"] = symbol_center.x;
        //     data["symbol_center_y"] = symbol_center.y;
        //     data["rotation_center_x"] = rotation_center.x;
        //     data["rotation_center_y"] = rotation_center.y;
        //     plotter.plot(data);
        // }

        bool was_reset = false;
        if (!fanblades.empty())
        {
            was_reset = solver.input(real_fanblade_c_point);
        }
        else
        {
            std::cout << "Warning: No fanblade detected, skip input to solver." << std::endl;
            // 可选：若连续多帧未检测到，可手动重置solver
            // static int no_detect_count = 0;
            // if (++no_detect_count > 20) { solver.reset(); no_detect_count = 0; }
        }

        if (was_reset)
        {
            std::cout << "Solver was reset due to camera movement. Re-learning..." << std::endl;
        }

        if (!fanblades.empty() && solver.isInitialized())
        {
            // a. 获取估计的中心点（米单位）
            cv::Point3f rotation_center = solver.getEstimatedCenter();

            // b. 3D旋转中心投影到图像平面（关键修改3：使用真实位姿）
            std::vector<cv::Point3f> pts3d = {rotation_center};
            std::vector<cv::Point2f> pts2d;
            // 注意：rotation_center是米单位，需转换为毫米（与buff_object_points一致）
            pts3d[0].x *= 1000.0;
            pts3d[0].y *= 1000.0;
            pts3d[0].z *= 1000.0;
            // 传入当前扇叶的rvec和tvec，而非零矩阵
            cv::projectPoints(pts3d, current_rvec, current_tvec, camera_matrix, distort_coeffs, pts2d);

            // c. 绘制旋转中心（半径用估计的半径，单位转换为像素时需注意，此处简化）
            cv::circle(display_img, pts2d[0], 8, cv::Scalar(255, 255, 0), -1);
            cv::putText(display_img, "Rotation Center",
                        cv::Point(pts2d[0].x + 10, pts2d[0].y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

            // d. PlotJuggler输出（单位统一为米）
            nlohmann::json data;
            data["symbol_center_x"] = real_fanblade_c_point.x;
            data["symbol_center_y"] = real_fanblade_c_point.y;
            data["symbol_center_z"] = real_fanblade_c_point.z;
            data["rotation_center_x"] = rotation_center.x;
            data["rotation_center_y"] = rotation_center.y;
            data["rotation_center_z"] = rotation_center.z;
            plotter.plot(data);
        }
        // 上面是gpt对于计算旋转中心的代码

        // plotjuggler

        // if (fanblades.size())
        // {
        //     // TODO
        // }
        // plotter.plot(data);

        // 显示检测结果
        cv::resize(display_img, display_img, {}, 0.8, 0.8);
        cv::imshow("Detection Results", display_img);
        if (cv::waitKey(30) == 27)
        { // 按ESC键退出
            break;
        }
    }
    return 0;
}