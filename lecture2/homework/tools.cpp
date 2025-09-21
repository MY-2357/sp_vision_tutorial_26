#include "tools.hpp"
#include <fmt/core.h>


// resize_image()  这个函数是我自己写的
// 输入任意尺寸的图像，将图像等比例放缩为640*640，
// 保证原图放缩后仍在图像中央，缺失的部分填充为黑色
bool resize_image(cv::Mat &input_image, cv::Mat &output_image)
{
    // 检查输入
    if (input_image.empty())
    {
        std::cout << "Input image is empty." << std::endl;
        return false;
    }
    // 判断图像的形状（width>=height还是height>width）
    bool is_width_larger = input_image.cols >= input_image.rows;
    // 计算放缩比例
    double scale = is_width_larger ? 640.0 / input_image.cols : 640.0 / input_image.rows;
    fmt::print("放缩比例：{:.2f}\n", scale);
    cv::Mat image_buffer;
    cv::resize(input_image, image_buffer, cv::Size(), scale, scale, cv::INTER_AREA);
    // 计算填充区域
    int x_offset = 0, y_offset = 0;
    if (is_width_larger)
        y_offset = (640 - image_buffer.rows) / 2;
    else
        x_offset = (640 - image_buffer.cols) / 2;
    fmt::print("图像偏移距离：x：{} y：{}\n", x_offset, y_offset);
    // 生成全黑图像，在上面进行覆盖
    output_image = cv::Mat::zeros(640, 640, CV_8UC3);
    image_buffer.copyTo(output_image(cv::Rect(x_offset, y_offset, image_buffer.cols, image_buffer.rows)));
    return true;
}

// 这个函数是在AI的帮助下写的，调用了cv::copyMakeBorder()函数。
bool resize_image_(cv::Mat &input_image, cv::Mat &output_image)
{

    if (input_image.empty())
        return false;

    double scale = 640.0 / std::max(input_image.cols, input_image.rows);
    fmt::print("放缩比例：{:2f}\n", scale);
    cv::Mat image_buffer;
    cv::resize(input_image, image_buffer, cv::Size(), scale, scale, cv::INTER_AREA);

    int x_offset = (640 - image_buffer.cols) / 2;
    int y_offset = (640 - image_buffer.rows) / 2;
    fmt::print("图像偏移距离：x：{} y：{}\n", x_offset, y_offset);

    cv::copyMakeBorder(image_buffer, output_image, y_offset, y_offset, x_offset, x_offset, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    return true;
}
