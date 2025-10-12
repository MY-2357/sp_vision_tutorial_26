#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/exiter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    // Your code start

    /*
      根据要求，需要循环进行以下步骤：
      1、打开相机并读取图像
      2、使用YOLO来检测并获取装机板的位置（像素坐标系，包括四个点）#暂时还不确定是只考虑装甲板只有一个的情况，还是考虑多个
      3、使用solvePnP来得出装甲板在相机坐标系下的位姿
      4、通过手眼标定实现相机坐标系与机器人本体坐标系的转换 #此步还不确定是否需要
      5、通过坐标变换将本体坐标系转换为世界坐标系 #这个应该是需要的，因而导致上一步也需要。因为发现
        「
        void send(bool control, bool fire, float yaw, float pitch); // 发送控制命令 其中yaw和pitch为世界坐标系中的绝对值
        」
        函数中，yaw和pitch为世界坐标系中的绝对值。
      6、从C板获取四元数来结算当前炮口的位姿
      7、需要计算出能够使云台移动后让炮口对准目标（装甲板的中心位置）的移动方式 #注意：此处对准的是中心位置，后续可能需要考虑到重力加速度的影响，不要忘了
      8、发送指令给云台使其转动到目标位置
    */

    // 打开相机并读取图像



    // Your code end
  }


  return 0;
}