#include "my_camera.hpp"

myCamera::myCamera()
{
    // 参数初始化
    m_handle_ = NULL;
    //----- 初始化相机状态-----

    // 查找可用相机
    int ret;
    MV_CC_DEVICE_INFO_LIST device_list;
    ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK || device_list.nDeviceNum == 0)
    {
        std::cout << "No camera found." << std::endl;
        return;
    }

    // 创建相机句柄
    ret = MV_CC_CreateHandle(&m_handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK)
    {
        std::cout << " Camera handle create failed." << std::endl;
        return;
    }

    // 打开相机
    ret = MV_CC_OpenDevice(m_handle_);
    if (ret != MV_OK)
    {
        std::cout << " Camera open failed." << std::endl;
        return;
    }

    // 设置相机参数（备注：这里理论上参数是需要在构造函数中传入的，但是由于个人也不知道什么参数合适，故采用原有的参数）
    MV_CC_SetEnumValue(m_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(m_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(m_handle_, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(m_handle_, "ExposureTime", 1000);
    MV_CC_SetFloatValue(m_handle_, "Gain", 20);
    MV_CC_SetFrameRate(m_handle_, 60);


}

myCamera::~myCamera()
{
    int ret;

    ret = MV_CC_CloseDevice(m_handle_);
    if (ret != MV_OK)
    {
        std::cout << "Error occurred when close device." << std::endl;
    }

    ret = MV_CC_DestroyHandle(m_handle_);
    if (ret != MV_OK)
    {
        std::cout << "Error occurred when destroy handle." << std::endl;
    }
}


bool myCamera::read(cv::Mat &img)
{
    //开始采集数据
    int ret = MV_CC_StartGrabbing(m_handle_);
    if (ret != MV_OK) 
    {
        std::cout << " Camera start grabbing failed." << std::endl;
        return false; 
    }

    if(!m_handle_)
    {
        std::cout<< "Camera handle is null." << std::endl;
        return false;
    }

    MV_FRAME_OUT raw;
    unsigned int nMsec = 100;

    ret = MV_CC_GetImageBuffer(m_handle_, &raw, nMsec);
    if (ret != MV_OK)
    {
        std::cout << "Error occurred when get image buffer." << std::endl;
        return false;
    }

    // 在这里获取需要的图像数据（根据获取的数据，调用MV_CC_FreeImageBuffer()后，img应该是可以正常访问的。但为了安全，使用了.clone()）
    img = transfer(raw).clone();

    ret = MV_CC_FreeImageBuffer(m_handle_, &raw);
    if (ret != MV_OK)
    {
        std::cout<< "Error occurred when free image buffer." << std::endl;
        return false;
    }

    ret = MV_CC_StopGrabbing(m_handle_);
    if (ret != MV_OK)
    {
        std::cout << "Error occurred when stop grabbing." << std::endl;
    }

    return true;
}


cv::Mat myCamera::transfer(MV_FRAME_OUT &raw)
{
    //这一段代码没有被使用

    // MV_CC_PIXEL_CONVERT_PARAM cvt_param;
     cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

    // cvt_param.nWidth = raw.stFrameInfo.nWidth;
    // cvt_param.nHeight = raw.stFrameInfo.nHeight;

    // cvt_param.pSrcData = raw.pBufAddr;
    // cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
    // cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

    // cvt_param.pDstBuffer = img.data;
    // cvt_param.nDstBufferSize = img.total() * img.elemSize();
    // cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    auto pixel_type = raw.stFrameInfo.enPixelType;
    const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
    cv::cvtColor(img, img, type_map.at(pixel_type));

    return img;
}

