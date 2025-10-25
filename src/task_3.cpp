#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/pid.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/trajectory.hpp"

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
  auto_aim::Aimer aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  /*
    根据要求，依次进行以下步骤：
    1、通过与task_1类似的方式获取当下装甲板的瞬时位姿（世界坐标系下）
    2、使用Target进行拟合，直到拟合的结果变为稳定状态
    3、获取Target中存储的每一个装甲板，根据拟合得到的角速度判断它到达云台与旋转中心连线所对应的角度的时间，
    4、再通过这个时间，得到该装甲板在这个时间之后所处的位置，
    5、计算弹丸击打到这个预测位置所需要的时间，如果这个时间和先前计算得到的旋转时间几乎相等，则发射弹丸对着那个位置进行射击
  */

  /*
    Target类分析
      1、Target类内部会直接使用扩展卡尔曼滤波器进行拟合，无需手动;
    Target接口分析
      1、 构造函数 Target(const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
        根据初始的armor进行构造，其中armor_num表示一个机器人上装甲板的数量，是固定的（应该是4），而radius，P0_dig只是给出一个初始值，后续会自动调整
      2、通过void update(const Armor & armor);传入新的数据点，用于更新模型
      3、bool convergened()是否已经收敛，没有收敛需要再等一等
      4、bool diverged() const;是否出现了发散（不收敛的时候也可能不发散），发散意味着模型出了问题，必须重新创建
      5、  void predict(std::chrono::steady_clock::time_point t);
          void predict(double dt);
          分别是通过输入绝对时间和相对时间来获取预测的状态
      6、  Eigen::VectorXd ekf_x() const;
           const tools::ExtendedKalmanFilter & ekf() const;
           ekf是背后的扩展卡尔曼滤波器，ekf_x是滤波器中的数据（共计11个自由度）
      7、std::vector<Eigen::Vector4d> armor_xyza_list() const;装甲板的位置列表，在使用predict之后使用

      听说是要先预测，再更新，还没有确定是不是
  */

  /*
    待处理：
    1、击打时，可能需要考虑云台转动时的速度对于轨迹的影响
    2、t似乎是在读取图片时更新的，指的是那一帧图像所对应的时间戳。但是由于我们的代码逻辑比较复杂，运行需要一定时间，
      可能会造成部分地方传入的t有些偏差。不过注意：并非所有的地方都要使用新的时间的，我们对原先图像上的装甲板的位置等信息进行分析时，
      肯定是要使用装甲板在那个时间下的状态。所以要修改时，请谨慎，看好每一个地方的t是干什么的
  */

  // 这是发送并且记录控制指令的环节，这个地方常用，而且很容易出问题，故而使用lambda表达式单独列出
  // 修改时，需要同时修改另外两个文件的对应函数
  auto send_command = [&gimbal, &plotter](
                        double yaw_target, double pitch_target, bool fire = false) -> void {
    gimbal.send(true, fire, yaw_target, pitch_target);
    // 使用plotter绘制向云台发送的控制信息
    nlohmann::json data;
    data["yaw"] = yaw_target;
    data["pitch"] = pitch_target;
    plotter.plot(data);
  };

  while (!exiter.exit()) {
    // 打开相机并读取图像
    camera.read(img, t);

    // 使用YOLO来检测并获取装甲板的位置（像素坐标系，包括四个点）
    auto armors = yolo.detect(img);
    if (cv::waitKey(20) == 'q') break;
    if (armors.empty()) {
      std::cout << "No Armor!" << std::endl;
      std::this_thread::sleep_for(100ms);
      continue;
    }

    // 使用Solver计算装甲板世界坐标
    q = gimbal.q(t);
    solver.set_R_gimbal2world(q);
    for (auto & armor : armors) {
      solver.solve(armor);
    }


    // 将装甲板信息传给 Aimer 进行自动拟合与击打判断
    std::list<auto_aim::Target> target_list;
    if (!armors.empty()) {
      // 以最高置信度装甲板为输入，生成 Target 列表
      auto best_armor = *std::max_element(
        armors.begin(), armors.end(), [](const auto_aim::Armor & a, const auto_aim::Armor & b) {
          return a.confidence < b.confidence;
        });
      target_list.emplace_back(best_armor, t, 0.2, 4, Eigen::VectorXd::Constant(11, 1.0));
    }

    // 调用 Aimer 自动处理拟合、预测、击打决策
    auto command = aimer.aim(target_list, t, gimbal.state().bullet_speed);

    if (command.control) {
      send_command(command.yaw, command.pitch, command.shoot);
      double predict_omega = target_list.front().ekf_x()[7];
      nlohmann::json data;
      data["predict_omega"] = predict_omega;  // 输出 EKF 预测的角速度
      plotter.plot(data);
    }
  }

  return 0;
}
