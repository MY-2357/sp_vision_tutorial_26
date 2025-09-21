#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

// 按照一定规则缩放图像
bool resize_image(cv::Mat& input_image, cv::Mat& output_image);
bool resize_image_(cv::Mat& input_image, cv::Mat& output_image);
#endif // TOOLS_HPP 