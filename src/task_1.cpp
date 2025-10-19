#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/pid.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

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
  tools::Exiter exiter;  //used
  tools::Plotter plotter;

  // 初始化io类
  io::Camera camera(config_path);  //used
  std::cout << "camera_gimial" << std::endl;
  io::Gimbal gimbal(config_path);  //used
  std::cout << "gimial_yolo" << std::endl;

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);  //used
  std::cout << "yolo_solver" << std::endl;
  auto_aim::Solver solver(config_path);  //used

  cv::Mat img;                              //used
  Eigen::Quaterniond q;                     //used
  std::chrono::steady_clock::time_point t;  //used

  // Your code start

  /*
      ../tasks/auto_aim中的各个类的分析：
      armor.hpp->Armor 装甲板，存储了装甲板的各个参数，以及自身在各个坐标系中的位置
      solver.hpp->Solver 进行各个坐标系之间的转换
      target.hpp->Target 传入数据，用于实现扩展卡尔曼滤波
      yolo.hpp->YOLO 用于实现装甲板的识别
    */

  /*
      ../io/gimbal/gimbal.hpp->Gimbal 云台控制类分析：
        void send(bool control, bool fire, float yaw, float pitch);
          方法中，control表示是否控制云台，应该设为true,本次fire为false,
          根据注释，yaw和pitch为世界坐标系中的绝对值，那应该在世界坐标系下进行计算发送指令。
          
        Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);
          中，这里的四元数q会随着云台的姿势的变化而变化，需要在每一次使用solver前对solver进行一次更新。
    */

  /*
      根据要求，
        控制云台转动跟踪装甲板应该是在机器人本体坐标系下完成。 
      循环进行以下步骤：
      1、打开相机并读取图像
      2、使用YOLO来检测并获取装甲板的位置（像素坐标系，包括四个点）#应该是只考虑一个装甲板
      3、从C板获取四元数来对要计算的世界坐标进行修正
      4、使用Solver来进行各个坐标系之间的转换，获取到装甲板的位置（本体坐标系）
      5、计算出能够使云台移动后让炮口对准目标（装甲板的中心位置）的移动方式 #注意：此处对准的是中心位置，后续可能需要考虑到重力加速度的影响，不要忘了
      6、使用PID控制算法，发送指令给云台使其稳定转动到目标位置
      7、使用plotter来绘制发送的信息
    */

  /*
      注意：可能需要一些格式的调整，以及注意之后添加错误处理
      没有使用tools/math_tools.hpp"，肯定在那个地方出了问题
    */

  /*
      已经发现，还没有解决的问题：
    */

  /*
      调试时需要做的：
      1、PID调参
          阶段	   调参目标           	方法
        ①先调 P 	快速响应	   增大 kp 直到系统略微震荡
        ②加 D	    抑制震荡	    增大 kd 直到平稳
        ③加 I	  消除稳态误差  	逐步增加 ki 直到误差趋零
        ④限幅调整	防止超出控制范围	调整 max_out、max_iout
      （上面的调参方法是GPT给的，可能需要根据实际情况调整）
    */

  // 这是发送并且记录控制指令的环节，这个地方常用，而且很容易出问题，故而使用lambda表达式单独列出
  // 修改时，需要同时修改另外两个文件的对应函数
  tools::PID pid_yaw(0.0f, 1.5f, 0.3f, 0.0f, 3.0f, 0.1f, true);
  tools::PID pid_pitch(0.0f, 1.5f, 0.3f, 0.0f, 3.0f, 0.1f, true);
  auto send_command = [&gimbal, &plotter, &pid_yaw, &pid_pitch](
                        double yaw_target, double pitch_target, bool fire = false) -> void {
    // if (pitch_target > 0.35)
    //   pitch_target = 0.35;
    // else if (pitch_target < -0.35)
    //   pitch_target = -0.35;
    //使用PID控制算法发送指令
    auto state = gimbal.state();  // 当前云台状态
    float yaw_output = pid_yaw.calc(yaw_target, state.yaw);
    float pitch_output = pid_pitch.calc(pitch_target, state.pitch);
    //  gimbal.send(true, fire, /*state.yaw +*/ yaw_output, /*state.pitch + */pitch_output);
    gimbal.send(true, fire, yaw_target, pitch_target);
    // 使用plotter绘制向云台发送的控制信息
    nlohmann::json data;
    // data["yaw"] = state.yaw + yaw_output;
    // data["pitch"] = state.pitch +  pitch_output;
    data["yaw"] = yaw_target;
    data["pitch"] = pitch_target;

    plotter.plot(data);
  };
  std::cout << "started" << std::endl;

  while (!exiter.exit()) {
    // 打开相机并读取图像
    camera.read(img, t);
    // 使用YOLO来检测并获取装甲板的位置（像素坐标系，包括四个点）
    auto armors = yolo.detect(img);
    if (cv::waitKey(20) == 'q') break;
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
    float yaw_target = tools::limit_rad(armor.ypd_in_world[0]);
    float pitch_target = -tools::limit_rad(armor.ypd_in_world[1]);

    nlohmann::json data;
    data["pos.x"] = pos[0];
    data["pos.y"] = pos[1];
    data["pos.z"] = pos[2];
    data["yaw_target"] = yaw_target;
    data["pitch_target"] = pitch_target;
    plotter.plot(data);
    // 调用send_command函数发送指令
    send_command(yaw_target, pitch_target);

    // Your code end
  }

  return 0;
}