#ifndef MY_CAMERA_HPP
#define MY_CAMERA_HPP

#include "hikrobot/include/MvCameraControl.h"
#include <opencv2/opencv.hpp>


/// @brief 相机类
/// @note 用于对工业相机进行操作并获取图像
class myCamera
{
public:
    //构造函数与析构函数
    myCamera();
    ~myCamera();

    //读取一帧图片
    bool read(cv::Mat& img);

protected:
    //tranfer方法:内部调用，把相机获取的原始数据转化为cv::Mat格式
    cv::Mat transfer(MV_FRAME_OUT& raw);
    //相机句柄
    void* m_handle_;
};

#endif // MY_CAMERA_HPP