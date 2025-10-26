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

  auto_aim::Target * pTarget = NULL;

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
  bool is_found_target;

  std::list<auto_aim::Target> target_list;
  /*
    注：因为得到的效果过于诡异，故而为了得到更高的分，我们这样进行：
    1、对前total_count次有效识别不做处理，只是用于计算出平均的omegea
    2、之后对omega进行判断，
    如果是在低转速(3-5rad),选择使用task_2中的代码进行射击（yaw或许可以微调一下）
    如果是在中高转速下，>5rad,选择使用我们最开始的模型进行射击(pitch需要调整)
    3、无论是在什么转速下，都使用aimer提供的omega进行输出。
  */

  int current_count = 0;
  int total_count = 100;  //计数前100次来判断当前的角速度。
  float former_omega[100] = {0};
  float guessed_omega = 0;
  int omega_recent_count = 0;
  int omaga_recent_total = 20;  // 计数最近20次来看omega有没有突变
  float recent_omega[20] = {0};
  bool is_checking_omega =
    false;  //感觉omega突变后变为true，停止射击，连续对omaga计数omaga_recent_total次，检查当前的omega是否正常
  int former_running_mode = 0;  //1&2
  int diverge_count_1 = 0;
  int diverge_count_2 = 0;
  int diverge_total = 10;  //模型连续发散十次后，重新构建模型
  int is_printed_model_converged = false;

  int plot_count = 0;
  int plot_sum = 20;
  float plot_data[20] = {0};
  bool is_calc_plot_omega_init = false;

  auto calc_plot_omega = [&plot_count, &plot_sum, &plot_data,
                          &is_calc_plot_omega_init](float input_omega) -> float {
    if (!is_calc_plot_omega_init) {
      for (int i = 0; i < plot_sum; i++) plot_data[i] = 0.0;
      is_calc_plot_omega_init = true;
    }
    // 每帧处理
    plot_data[plot_count] = input_omega;
    plot_count++;
    if (plot_count >= plot_sum) plot_count = 0;
    // 计算出要输出的数据
    float sum;
    for (int i = 0; i < plot_sum; i++) {
      sum += plot_data[i];
    }
    float result = sum / plot_sum;
    return result;
  };

  while (!exiter.exit()) {
    // Your code start

    // 打开相机并读取图像
    camera.read(img, t);
    // 使用YOLO来检测并获取装甲板的位置（像素坐标系，包括四个点）
    auto armors = yolo.detect(img);
    if (cv::waitKey(20) == 'q') break;
    if (armors.empty())  // 没有检测到装甲板
    {
      // 让线程休眠，减少资源占用
      //  std::cout << "No Armor!" << std::endl;
      //std::this_thread::sleep_for(100ms);
      continue;
    }
    // 这个地方本来想要依次传入每一个装甲板进行拟合的，但是豆包说一帧传入多个容易出问题，故而每次传入可信度最高的装甲板
    auto best_armor_it = armors.begin();  // 指向可信度最高的装甲板
    for (auto it = armors.begin(); it != armors.end(); ++it) {
      // Solver计算armor的位置
      q = gimbal.q(t);               //获取当前时间戳下云台的姿态
      solver.set_R_gimbal2world(q);  // 使用从C板获取的四元数来对solver计算的世界坐标加以修正
      solver.solve(*it);
      // 比较置信度，更新最佳装甲板的迭代器
      if (it->confidence > best_armor_it->confidence) best_armor_it = it;
    }
    auto & best_armor = *best_armor_it;  // 指向可信度最高的装甲板

    /***************section1***** */

    // 将装甲板信息传给 Aimer 进行自动拟合与击打判断
    if (!armors.empty()) {
      // 以最高置信度装甲板为输入，生成 Target 列表
      auto best_armor = *std::max_element(
        armors.begin(), armors.end(), [](const auto_aim::Armor & a, const auto_aim::Armor & b) {
          return a.confidence < b.confidence;
        });
      // target_list.emplace_back(best_armor, t, 0.2, 4, Eigen::VectorXd::Constant(11, 1.0));
      // Target实例管理：仅保留一个实例并更新
      if (target_list.empty()) {
        // 首次创建：参数可根据实际调整（radius为旋转半径，P0_dig为初始协方差）
        target_list.emplace_back(best_armor, t, 0.2, 4, Eigen::VectorXd::Constant(11, 1.0));
      } else {
        auto & target = target_list.front();  // 操作唯一实例
        if (target.diverged()) {
          diverge_count_1++;
          if (diverge_count_1 >= diverge_total) {
            // 若发散，重建实例
            target_list.clear();
            target_list.emplace_back(best_armor, t, 0.2, 4, Eigen::VectorXd::Constant(11, 1.0));
            diverge_count_1 = 0;
          }
        } else {
          // 未发散，更新EKF（关键步骤）
          target.update(best_armor);
          // 预测到当前时间，确保状态同步
          target.predict(t);
        }
      }
    } else {
      //  std::cout << "No Armor!" << std::endl;
      target_list.clear();  // 无装甲板时清空旧实例，避免后续用旧数据更新
  //    std::this_thread::sleep_for(100ms);
      continue;
    }

    // 调用 Aimer 自动处理拟合、预测、击打决策
    auto command = aimer.aim(target_list, t, gimbal.state().bullet_speed);
    if (current_count < 100) {
      if (command.control) {
        // send_command(command.yaw, command.pitch, command.shoot);
        double predict_omega = target_list.front().ekf_x()[7];
        nlohmann::json data;
        data["predict_omega"] = calc_plot_omega(predict_omega);  // 输出 EKF 预测的角速度
        plotter.plot(data);
        former_omega[current_count] = predict_omega;
        current_count++;
        continue;
      }
    } else {
      if (!guessed_omega) {
        float sum = 0.0;
        for (int i = 0; i < total_count; i++) sum += former_omega[i];
        guessed_omega = sum / total_count;
        std::cout << "guessed_omega:" << guessed_omega << std::endl;
      }
    }
    // 这里对得到的角速度进行检查，因为测试的时候会出现角速度的档位切换的情况，所以要看一下是否切换了档位
    if (abs(target_list.front().ekf_x()[7] - guessed_omega) > 2.0)  //得到的omega超出阈值
    {
      if (!is_checking_omega) {
        std::cout << "omaga may be changed,detecting..." << std::endl;
        is_checking_omega = true;
      }
    }
    if (is_checking_omega) {
      if (omega_recent_count < omaga_recent_total) {
        recent_omega[omega_recent_count] = target_list.front().ekf_x()[7];
        omega_recent_count++;
      } else {
        float sum = 0.0;
        for (int i = 0; i < omaga_recent_total; i++) sum += recent_omega[i];
        sum = sum / omaga_recent_total;
        if (abs(sum - guessed_omega) > 0.5)  //omega超出阈值，发生突变，重新计算guessed_omega
        {
          current_count = 0;
          guessed_omega = 0.0;
          for (int i = 0; i < total_count; i++) former_omega[i] = 0.0;
          std::cout << "omega changed, reseting guessed_omega..." << std::endl;
          omega_recent_count = 0;
          is_checking_omega = false;
          continue;
        } else {
          //没有出现问题 ，可以继续根据原来的方案射击
          is_checking_omega = false;
          omega_recent_count = 0;
        }
      }
    }

    //现在，我们已经得到的预测的当前角速度，可以进行下一步了。
    if (abs(guessed_omega) < 5.0)  // 低速档
    {
      if (former_running_mode != 1) {
        std::cout << "running code of task_2" << std::endl;
        former_running_mode = 1;
      }
      //直接使用task_1的代码
      auto_aim::Armor armor = armors.front();  //只跟随第一个装甲板
      // Solver计算armor的位置
      q = gimbal.q(t);               //获取当前时间戳下云台的姿态
      solver.set_R_gimbal2world(q);  // 使用从C板获取的四元数来对solver计算的世界坐标加以修正
      solver.solve(armor);
      //计算pitch和yaw
      float yaw_target = armor.ypd_in_world.x();
      float pitch_target = -armor.ypd_in_world.y();

      /*****************上面代码与task_1基本相同************/
      // 计算云台的合适朝向
      auto pos_xyz = armor.xyz_in_world;  // 装甲板的中心位置
      tools::Trajectory trajectory(   //备注：这一行可能会出错的地方：我们认为pos_xyz.z()是对应的目标与跑口的相对高度，但实际上我们并不难肯定炮口处的z值为0
      gimbal.state().bullet_speed, sqrt(pos_xyz.x() * pos_xyz.x() + pos_xyz.y() * pos_xyz.y()), pos_xyz.z());
      if (trajectory.unsolvable)  //当前距离无法射击
      {
        // 无法射击也要去调节云台的朝向，因为云台的yaw必须对准，pitch指向装甲板中心的话，误差倒不大
        send_command(yaw_target, pitch_target);
        std::cout << "to far to reach" << std::endl;
        continue;
      }
      pitch_target = -trajectory.pitch;  //更新pitch_target
      // 检查是否符合射击条件
      nlohmann::json data_tmp;
      data_tmp["d_pitch"] = gimbal.state().pitch - pitch_target;
      data_tmp["d_yaw"] = gimbal.state().yaw - yaw_target;
      data_tmp["gimbal_pitch"] = gimbal.state().pitch;
      data_tmp["gimbal_yaw"] = gimbal.state().yaw;

      plotter.plot(data_tmp);

      if (
        abs(gimbal.state().pitch - pitch_target) <
          0.015 &&  //射击条件这里其实也不太清楚，目前限制为当前状态与目标状态的yaw与pitch
        abs(gimbal.state().yaw - yaw_target) < 0.015)  //阈值为相差在0.05rad之内，之后肯定需要调
      {
        // 符合射击条件，发送射击指令
        pitch_target += 0.008;
        yaw_target -= 0.005;
        send_command(yaw_target+0.028, pitch_target, true);
        send_command(yaw_target+0.028, pitch_target, false);
        // send_command(yaw_target, pitch_target, false);
        std::cout << "fire" << std::endl;
        // 等待一段时间，然后重新开始循环
 //       std::this_thread::sleep_for(200ms);
        continue;
      }
      send_command(yaw_target+0.028, pitch_target);
      // 计算并输出omega (注意：此时的omega 的误差会比较大，我们加一个平均)
      nlohmann::json data;
      data["predict_omega"] =
        calc_plot_omega(guessed_omega * 0.7 + target_list.front().ekf_x()[7] * 0.3);
      plotter.plot(data);
    } else if (abs(guessed_omega) >= 5.0)  //中高速档
    {
      if (former_running_mode != 2) {
        std::cout << "running task_3" << std::endl;
        former_running_mode = 2;
      }
      //直接使用原来的代码
      if (!pTarget)  //第一次检测到装甲板时，创建Target对象
        pTarget = new auto_aim::Target(best_armor, t, 0.2, 4, Eigen::VectorXd::Constant(11, 1.0));
      pTarget->predict(t);          //传入时间戳
      pTarget->update(best_armor);  //更新Target对象
      //   pTarget->update(armors.front());

      // 未检测到装甲板时，pTarget为NULL，代码不能继续执行，而是选择等待
      if (!pTarget) continue;

      if (pTarget->diverged())  //模型出现了发散，必须重新创建Target对象进行拟合
      {
        diverge_count_2++;
        if (diverge_count_2 >= diverge_total) {
          std::cout << "The model is diverged...restarting" << std::endl;
          delete pTarget;
          send_command(best_armor.ypd_in_world[0], best_armor.ypd_in_world[1]);
          pTarget = new auto_aim::Target(best_armor, t, 0.2, 4, Eigen::VectorXd::Constant(11, 1.0));
          diverge_count_2 = 0;
          is_printed_model_converged = false;
        }
        continue;
      }

      if (!pTarget->convergened()) continue;  //模型还未收敛，继续等待
      //模型已经收敛
      if (!is_printed_model_converged) {
        std::cout << "The model is converged!" << std::endl;
        is_printed_model_converged = true;
      }
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
      data["predict_omega"] = calc_plot_omega(target_list.front().ekf_x()[7]);
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
        time_to_shoot =
          (tools::limit_rad(yaw_target) - tools::limit_rad(target_List[armor_id][3])) /
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
        if (abs(tools::limit_rad(predict_target[3]) - yaw_target) >= 0.25)  //设置检测阈值为0.25rad
          continue;                                                        //不符合要求
        // 计算打击所需要的时间，判断是否可以击打
        tools::Trajectory trajectory(
          gimbal.state().bullet_speed,
          sqrt(predict_target[0] * predict_target[0] + predict_target[1] * predict_target[1]),
          predict_target
            [2]);  //备注：这一行可能会出错的地方：我们认为pos_xyz.z()是对应的目标与跑口的相对高度，但实际上我们并不难肯定炮口处的z值为0
        if (trajectory.unsolvable) continue;
        if (abs(trajectory.fly_time - time_to_shoot) >= 0.02)
          continue;  //设置阈值为0.05s,转动到目标位置所需要的时间与预测飞行时间相差过大认为无法击打
        // 可以击打，发送指令,对着装甲板的预测位置击打（这会导致云台有着轻微的持续转动）
        send_command(predict_target[3], trajectory.pitch+0.08, true);
        pTarget->predict(t);                 // 更新预测状态
 //       std::this_thread::sleep_for(200ms);  // 延时200ms，等待云台稳定
        continue;
      }
    } else  //如果出现其他情况，肯定是说明出了问题，需要重新计算guessd_omega
    {
      current_count = 0;
      guessed_omega = 0.0;
      for (int i = 0; i < total_count; i++) former_omega[i] = 0.0;
      continue;
    }
  }

  if (pTarget) delete pTarget;

  return 0;
}