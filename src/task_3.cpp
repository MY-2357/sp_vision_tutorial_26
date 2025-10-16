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
    3、根据估计状态，在考虑重力加速度的情况下，调整云台的姿态，直到调节稳定后，不再移动云台，保持云台静止
    4、静止后，再次使用Target进行拟合，得到目标的运动的最终方程以及可以击打目标的时机
    5、考虑子弹在空中运动的时间以及其他可能的时间（如发射子弹需要的时间，得到可以发射子弹的时机）
    6、在到达时机后，发射子弹击打目标，持续十次，并在极大的过程中更新预测
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
    可能需要修改的：
      1、装甲板数据一次可以传入多个
  */

  //辅助函数
  auto get_r=[]()->float{
    //获取装甲板到旋转中心的距离
    
  };

  auto_aim::Target * pTarget = NULL;

  //云台中心（世界坐标原点）与旋转中心的连线的角度
  float to_C_yaw = 666.0;                        //初始化前随便设置一个角度，不在可转动的角度内
  bool is_found_target = false;                  //是否已经得到了需要的大致装甲板位置
  std::list<Eigen::Vector4d> armor_target_list;  //需要攻击的装甲板位置（4维向量），x,y,z,w

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
    auto_aim::Armor armor =
      armors
        .front();  //只跟随第一个装甲板（注：这个地方为了增加数据点的个数，应该传入所有的数据的，不过等调试后再说）
    // Solver计算armor的位置
    q = gimbal.q(t);               //获取当前时间戳下云台的姿态
    solver.set_R_gimbal2world(q);  // 使用从C板获取的四元数来对solver计算的世界坐标加以修正
    solver.solve(armor);
    /*****************上面代码与task_1相同************/
    if (!pTarget)  //第一次检测到装甲板时，创建Target对象
      pTarget = new auto_aim::Target(armor, t, 0.2, 4, Eigen::VectorXd::Constant(11, 1.0));
    pTarget->update(armor);   //更新Target对象
    if (pTarget->diverged())  //模型出现了发散，必须重新创建Target对象进行拟合
    {
      delete pTarget;
      pTarget = new auto_aim::Target(armor, t, 0.2, 4, Eigen::VectorXd::Constant(11, 1.0));
      continue;
    }
    if (pTarget->convergened())  //模型已经收敛
    {
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
      // 发生命令调整云台
      // 这个调整过程十分复杂，我们先让云台对准旋转中心，然后再进行微调
      auto curren_ekf_x = pTarget->ekf_x();
      Eigen::Vector3d rotation_C_world(
        curren_ekf_x[0], curren_ekf_x[2], curren_ekf_x[4]);  //旋转中心在世界坐标系下坐标
      // send_command(target_ypd[0], target_ypd[1], false);
      // 要想得到一定时间后的目标位置，需要先有时间，但是时间只有通过位置才能得到，所以我们就只能先进行一个估计
      auto rotation_C_world_ypd = tools::xyz2ypd(rotation_C_world);
      if (to_C_yaw == 666.0) {
        to_C_yaw = rotation_C_world_ypd
          [0];  //通过云台中心与旋转中的连线的角度来进行预测，我们获取位于连线上的装甲板位置
        continue;
      }
      else
      {}
      auto get_yaw_from_xyz = [](float x, float y, float z) -> float {
        return tools::xyz2ypd(Eigen::Vector3d(x, y, z))[0];
      };
      // 首先转换参考系为旋转中心
      auto predict_armor_list = pTarget->armor_xyza_list();
      auto target_armor = predict_armor_list.front();
      //反之，则之前已经设置过to_C_yaw,我们会在误差允许的范围内进行微调
      Eigen::Vector4d armor_tmp;
      if (!is_found_target) 
      {
        for (Eigen::Vector4d armor_tmp : predict_armor_list) 
        {
          if (
            abs(get_yaw_from_xyz(armor_tmp[0], armor_tmp[1], armor_tmp[2]) - to_C_yaw) <
            0.1)  //设置阈值（小于0.1rad时认为为可以用来打击的装甲板）
          {
            // 先遍历已有链表，查看链表中是否有位置相近的装甲板(认为是同一个装甲板，或者至少是同一类型的，击打时不需要区分)
            bool is_added=false;
            for(Eigen::Vector4d armor_target : armor_target_list)
            {
                // 我们通过装甲板到旋转中心的距离以及装甲板的所处高度来判断是否是同一个装甲板

              };
            }
            armor_target_list.push_back(armor_tmp);
            is_found_target = true;
          }
        }
      }
      Eigen::Vector3d armor_C_world(
        target_armor[0], target_armor[1], target_armor[2]);  //取出的第一个装甲板在世界坐标系下坐标
      auto armor_C_rotation_C =
        armor_C_world - rotation_C_world;  //装甲板相对于旋转中心的坐标(笛卡尔坐标系)
      auto armor_C_rotation_C_ypd = tools::xyz2ypd(armor_C_rotation_C);  //转换为球坐标系

      auto xyz_in_rotation_c =
        /*
      armor_xyza_list链表中的每个向量的维度是x,y,z,w，暂时认为是相对于世界坐标系原点来说的（当然w不是）

      */

        // 使用plotter绘制向云台发送的ekf拟合得到目标角速度
        nlohmann::json data;
      data["predicted_omega"] = curren_ekf_x(7);
      plotter.plot(data);
    }
    // Your code end
  }

  if (pTarget) delete pTarget;

  return 0;
}