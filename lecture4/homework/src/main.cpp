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

            cv::Mat rmat;
            cv::Rodrigues(rvec, rmat);
            double yaw = atan2(rmat.at<double>(0, 2), rmat.at<double>(2, 2));
            double pitch = -asin(rmat.at<double>(1, 2));
            double roll = atan2(rmat.at<double>(1, 0), rmat.at<double>(1, 1));
            tools::draw_text(display_img, fmt::format("yaw:   {:.2f}", yaw * 180 / 3.1415926), cv::Point2f(10, 180), 1.7, cv::Scalar(0, 255, 255), 3);
            tools::draw_text(display_img, fmt::format("pitch: {:.2f}", pitch * 180 / 3.1415926), cv::Point2f(10, 240), 1.7, cv::Scalar(0, 255, 255), 3);
            tools::draw_text(display_img, fmt::format("roll:  {:.2f}", roll * 180 / 3.1415926), cv::Point2f(10, 300), 1.7, cv::Scalar(0, 255, 255), 3);

            // 添加每一个扇叶中心的数据
            if (fanblades.size())
            {
                std::string fanblade_center_x_index = "fanblade_" + std::to_string(count) + "_c_x";
                std::string fanblade_center_y_index = "fanblade_" + std::to_string(count) + "_c_y";
                std::string fanblade_center_z_index = "fanblade_" + std::to_string(count) + "_c_z";

                data[fanblade_center_x_index] = tvec.at<double>(0);
                data[fanblade_center_y_index] = tvec.at<double>(1);
                data[fanblade_center_z_index] = tvec.at<double>(2);

                cv::Point3f rotation_C;

                auto_buff::calculateRotationCenter(rvec, tvec,rotation_C);
                tools::draw_text(display_img, fmt::format("rotation_C:  x{: .2f} y{: .2f} z{: .2f}", rotation_C.x, rotation_C.y, rotation_C.z), cv::Point2f(10, 360), 1.7, cv::Scalar(0, 255, 255), 3);
                data["rotation_C_x"]=rotation_C.x;
                data["rotation_C_y"]=rotation_C.y;
                data["rotation_C_z"]=rotation_C.z;
            }

            count++;
        }

        plotter.plot(data);

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
