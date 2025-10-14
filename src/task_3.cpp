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

  /*
    根据要求，依次进行以下步骤：
    1、通过与task_1类似的方式获取当下装甲板的瞬时位姿（世界坐标系下）
    2、使用Target进行拟合，输出目标的估计状态
    3、根据估计状态，在考虑重力加速度的情况下，调整云台的姿态，直到调节稳定后，不再移动云台，保持云台静止
    4、静止后，再次使用Target进行拟合，得到目标的运动的最终方程以及可以击打目标的时机
    5、考虑子弹在空中运动的时间以及其他可能的时间（如发射子弹需要的时间，得到可以发射子弹的时机）
    6、在到达时机后，发射子弹击打木目标，持续十次，并在极大的过程中更新预测
  */


  while (!exiter.exit()) {
    // Your code start

    /************下面的代码与task_1相同************/
    // 打开相机并读取图像
    camera.read(img, t);
    // 使用YOLO来检测并获取装甲板的位置（像素坐标系，包括四个点）
    auto armors = yolo.detect(img);
    if (armors.empty())  // 没有检测到装甲板
    {
      // 让线程休眠，减少资源占用
      std::this_thread::sleep_for(100ms);
      continue;
    }
    auto_aim::Armor armor = armors.front();  //只跟随第一个装甲板
    // Solver计算armor的位置
    q = gimbal.q(t);               //获取当前时间戳下云台的姿态
    solver.set_R_gimbal2world(q);  // 使用从C板获取的四元数来对solver计算的世界坐标加以修正
    solver.solve(armor);
    //计算pitch和yaw
    auto pos = armor.xyz_in_world;  // 装甲板的中心位置
    /*
      这里使用了一个假设：yaw，pitch的取值是负pi到pi，
      yaw是从x开始，以z为转轴转动逆时针的角度，pitch是从x开始，以y为转轴转动逆时针的角度
    */
    float yaw_target = std::atan2(pos.y(), pos.x());
    float pitch_target = -std::atan2(pos.z(), pos.x());
    // 这是发送并且记录控制指令的环节，这个地方常用，而且很容易出问题，故而使用lambda表达式单独列出
    // 修改时，需要同时修改另外两个文件的对应函数
    auto send_command = [&gimbal, &plotter](
                          int yaw_target, int pitch_target, bool fire = false) -> void {
      //使用PID控制算法发送指令
      tools::PID pid_yaw(0.01f, 5.0f, 0.0f, 0.5f, 5.0f, 0.2f, true);
      tools::PID pid_pitch(0.01f, 5.0f, 0.0f, 0.5f, 5.0f, 0.2f, true);
      auto state = gimbal.state();  // 当前云台状态
      float yaw_output = pid_yaw.calc(yaw_target, state.yaw);
      float pitch_output = pid_pitch.calc(pitch_target, state.pitch);
      gimbal.send(true, fire, state.yaw + yaw_output, state.pitch + pitch_output);
      // 使用plotter绘制向云台发送的控制信息
      nlohmann::json data;
      data["yaw"] = state.yaw + yaw_output;
      data["pitch"] = state.pitch + pitch_output;
      plotter.plot(data);
    };
    /*****************上面代码与task_1相同************/
  

    // Your code end
  }


  return 0;
}