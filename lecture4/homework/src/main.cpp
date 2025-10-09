#include "tasks/buff_detector.hpp"
#include "io/camera.hpp"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include "tools/plotter.hpp"
#include "tasks/buff_solver.hpp"
#include "tools/img_tools.hpp"
#include "fmt/core.h"


int main()
{

    // 下面的代码调试用
    cv::VideoCapture cap("assets/test.avi");

    if (!cap.isOpened())
    {
        std::cerr << "无法打开视频文件!" << std::endl;
        return -1;
    }

    // io::Camera camera(2.5, 16.9, "2bdf:0001");
    std::chrono::steady_clock::time_point timestamp;
    auto_buff::Buff_Detector detector;
    tools::Plotter plotter;

    auto_buff::Buff_Solver solver;
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

        // camera.read(img, timestamp);
        // auto fanblades = detector.detect(img);

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

            tools::draw_text(display_img, fmt::format("tvec:  x{: .2f} y{: .2f} z{: .2f}", tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)), cv::Point2f(10, 60), 1.7, cv::Scalar(0, 255, 255), 3);
            tools::draw_text(display_img, fmt::format("rvec:  x{: .2f} y{: .2f} z{: .2f}", rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)), cv::Point2f(10, 120), 1.7, cv::Scalar(0, 255, 255), 3);

            // 添加每一个扇叶中心的数据
            if (fanblades.size())
            {
                std::string fanblade_center_x_index = "fanblade_" + std::to_string(count) + "_c_x";
                std::string fanblade_center_y_index = "fanblade_" + std::to_string(count) + "_c_y";
                std::string fanblade_center_z_index = "fanblade_" + std::to_string(count) + "_c_z";

                data[fanblade_center_x_index] = tvec.at<double>(0);
                data[fanblade_center_y_index] = tvec.at<double>(1);
                data[fanblade_center_z_index] = tvec.at<double>(2);

                if (count == 0)
                {
                    real_fanblade_c_point.x = tvec.at<double>(0);
                    real_fanblade_c_point.y = tvec.at<double>(1);
                    real_fanblade_c_point.z = tvec.at<double>(2);
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

        if (!fanblades.empty())
        {

            // a. 获取估计的中心点
            cv::Point3f symbol_center = real_fanblade_c_point;                  // solvePnP得到的3D坐标
            cv::Point3f rotation_center = solver.updateAndSolve(symbol_center); // 调用solver得到的2D坐标

            // 投影到图像平面
            std::vector<cv::Point3f> pts3d = {rotation_center};
            std::vector<cv::Point2f> pts2d;
            cv::projectPoints(pts3d, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), camera_matrix, distort_coeffs, pts2d);

            // 绘制
            //   cv::circle(display_img, pts2d[0], 8, cv::Scalar(255, 255, 0), -1);

            nlohmann::json data;
            data["symbol_center_x"] = symbol_center.x;
            data["symbol_center_y"] = symbol_center.y;
            data["symbol_center_z"] = symbol_center.z;
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
