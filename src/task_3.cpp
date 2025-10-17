#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
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

  auto_aim::Target * pTarget = NULL;

  // 这是发送并且记录控制指令的环节，这个地方常用，而且很容易出问题，故而使用lambda表达式单独列出
  // 修改时，需要同时修改另外两个文件的对应函数
  tools::PID pid_yaw(0.01f, 5.0f, 0.0f, 0.5f, 5.0f, 0.2f, true);
  tools::PID pid_pitch(0.01f, 5.0f, 0.0f, 0.5f, 5.0f, 0.2f, true);
  auto send_command = [&gimbal, &plotter, &pid_yaw, &pid_pitch](
                        double yaw_target, double pitch_target, bool fire = false) -> void {
    //使用PID控制算法发送指令
    auto state = gimbal.state();  // 当前云台状态
    double yaw_output = pid_yaw.calc(yaw_target, state.yaw);
    double pitch_output = pid_pitch.calc(pitch_target, state.pitch);
    gimbal.send(true, fire, state.yaw + yaw_output, state.pitch + pitch_output);
    // 使用plotter绘制向云台发送的控制信息
    nlohmann::json data;
    data["yaw"] = state.yaw + yaw_output;
    data["pitch"] = state.pitch + pitch_output;
    plotter.plot(data);
  };

  while (!exiter.exit()) {
    // Your code start

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
    // 这个地方本来想要依次传入每一个装甲板进行拟合的，但是豆包说一帧传入多个容易出问题，故而每次传入可信读最高的装甲板
    auto best_armor_it = armors.begin();  // 指向可信度最高的装甲板
    for (auto it = armors.begin(); it != armors.end(); ++it) {
      // Solver计算armor的位置
      q = gimbal.q(t);               //获取当前时间戳下云台的姿态
      solver.set_R_gimbal2world(q);  // 使用从C板获取的四元数来对solver计算的世界坐标加以修正
      solver.solve(*it);
      // 比较置信度，更新最佳装甲板的迭代器
      if (it->confidence > best_armor_it->confidence)  // it-> 访问成员（类似指针->）
        best_armor_it = it;
    }
    auto & best_armor = *best_armor_it;  // 指向可信度最高的装甲板

    if (!pTarget)  //第一次检测到装甲板时，创建Target对象
      pTarget = new auto_aim::Target(best_armor, t, 0.2, 4, Eigen::VectorXd::Constant(11, 1.0));
    pTarget->predict(t);  //传入时间戳
    pTarget->update(best_armor);  //更新Target对象

    // 未检测到装甲板时，pTarget为NULL，代码不能继续执行，而是选择等待
    if (!pTarget) continue;

    if (pTarget->diverged())  //模型出现了发散，必须重新创建Target对象进行拟合
    {
      delete pTarget;
      pTarget = new auto_aim::Target(best_armor, t, 0.2, 4, Eigen::VectorXd::Constant(11, 1.0));
      continue;
    }
    if (!pTarget->convergened()) continue;  //模型还未收敛，继续等待
    //模型已经收敛

    // 先获取当下云台中心与旋转中心水平连线的角度
    auto rotation_C_info = pTarget->ekf_x();
    /*
      x vx y vy z vz a w r l h
      a: angle
      w: angular velocity
      l: r2 - r1
      h: z2 - z1
    */
    // 使用plotter绘制向云台发送的控制信息
    nlohmann::json data;
    data["predict_omega"]=pTarget->ekf_x()[7];
    plotter.plot(data);
    //
    auto yaw_target = atan2(rotation_C_info[2], rotation_C_info[0]);
    auto target_List = pTarget->armor_xyza_list();
    /*
        x y z a
        x,y,z:向对于世界坐标系原点
        a:转动角度，以世界坐标系x轴为基准，顺时针为负，单位为弧度
    */
    //遍历每一个装甲板，检查目前是否适合射击,可以便选择射击
    for (int armor_id = 0; armor_id < target_List.size(); armor_id++) {
      // 计算弹丸击打到预测位置所需要的时间
      double time_to_shoot;
      time_to_shoot = (tools::limit_rad(yaw_target) - tools::limit_rad(target_List[armor_id][3])) /
                      rotation_C_info[7];
      if (time_to_shoot < 0)
        time_to_shoot +=
          (2 * M_PI) /
          abs(
            rotation_C_info
              [7]);  //这个地方是考虑到了：时间有可能为负，在这种情况下，旋转的角速度也可能是负的
      // 计算到预测时间后，装甲板的位置
      pTarget->predict(time_to_shoot);
      auto predict_target = pTarget->armor_xyza_list()[armor_id];
      pTarget->predict(t);  // 回退为原来的时间
      // 先检查一下预测的装甲板位置是否在yaw_target附近
      if (abs(tools::limit_rad(predict_target[3]) - yaw_target) >= 0.1)  //设置检测阈值为0.1rad
        continue;                                                        //不符合要求
      // 计算打击所需要的时间，判断是否可以击打
      tools::Trajectory trajectory(
        gimbal.state().bullet_speed,
        sqrt(predict_target[0] * predict_target[0] + predict_target[1] * predict_target[1]),
        predict_target
          [2]);  //备注：这一行可能会出错的地方：我们认为pos_xyz.z()是对应的目标与跑口的相对高度，但实际上我们并不难肯定炮口处的z值为0
      if (trajectory.unsolvable) continue;
      if (abs(trajectory.fly_time - time_to_shoot) >= 0.05)
        continue;  //设置阈值为0.05s,转动到目标位置所需要的时间与预测飞行时间相差过大认为无法击打
      // 可以击打，发送指令,对着装甲板的预测位置击打（这会导致云台有着轻微的持续转动）
      send_command(predict_target[3],trajectory.pitch,true);
      pTarget->predict(t);  // 更新预测状态
      std::this_thread::sleep_for(500ms);   // 延时500ms，等待云台稳定
      continue;
    }
  }

  if (pTarget) delete pTarget;

  return 0;
}