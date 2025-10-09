#ifndef TOOLS__IMG_TOOLS_HPP
#define TOOLS__IMG_TOOLS_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace tools
{
void draw_point(
  cv::Mat & img, const cv::Point & point, const cv::Scalar & color = {0, 0, 255}, int radius = 3);

    // 为了方便调试，从class/tools文件夹中的img_tools.hpp中复制了一份draw_text函数

    // 绘图函数，依次传入：图像、字符串、显示位置
    inline void draw_text(
        cv::Mat &img, const std::string &text, const cv::Point &point, double font_scale = 1.0,
        const cv::Scalar &color = cv::Scalar(0, 255, 255), int thickness = 2)
    {
        cv::putText(img, text, point, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
    }


}  // namespace tools

#endif  // TOOLS__IMG_TOOLS_HPP