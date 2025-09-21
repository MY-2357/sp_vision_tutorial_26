#include "tools.hpp"
#include <fmt/core.h>

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input_image>\n";
        return 1;
    }
    std::string inPath = argv[1];
    fmt::print("Input image: {}\n", inPath);
    // 读取图片
    cv::Mat img = cv::imread(inPath);
    // 调用resize_image()函数对图片进行缩放
    cv::Mat resized_img;
    //resize_image(img, resized_img);
    resize_image_(img, resized_img);
    cv::imshow("Resized Image", resized_img);
    cv::waitKey(0); 
    return 0;
}