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
    ../tools/中的各个模块分析
    crc.hpp: CRC校验算法，用于校验数据传输是否正确
    extended_kalman_filter.hpp : 扩展卡尔曼滤波器
    img_tools.hpp：图像处理工具，实现在图像上绘点、输出文字
    logger.hpp：向控制台输出日志，同时保存到文件
    math_tools.hpp：实现坐标系的转换与其他一些常见数学计算
    pid.hpp->PID: PID控制算法,实现稳定控制机器人的运动
    ransac_sine_fitter.hpp->RansacSineFitter：正弦函数拟合器，从充满噪声的点集中估计出一条最佳拟合曲线
    recorder.hpp->Recorder：根据连续传入的图片按照制定的帧率录制视频，同时将每帧对应的姿态数据（四元数）保存到文件
    thread_pool.hpp->ThreadPool：线程池，用于并行执行任务
    thread_pool.hpp->OrderedQueue:实现一个按顺序输出的队列，即使输入乱序，也能按 id 顺序输出。
    thread_safe_queue.hpp->ThreadSafeQueue：模板类，提供线程安全的生产者-消费者队列
    trajectory.hpp->Trajectory：用于计算在考虑重力加速度，忽略空气阻力的情况下对象所需要的条件
  */

  /*
    上述的模块的应用分析：
    pid.hpp->PID：
      实现稳定控制机器人的运动，在任何控制机器人运动的地方都要想着使用它。
    thread_pool.hpp->OrderedQueue：
      在相机图像流（或YOLO检测结果）中，帧可能乱序到达：
      OrderedQueue 负责重新排序；
      让后续模块始终按正确的时间顺序处理帧。
  */

  /*
    根据要求，依次进行以下步骤：
    1、通过与task_1类似的方式获取当下装甲板的位姿（世界坐标系下）
    2、考虑重力加速度，使用Trajectory计算云台的合适朝向
    3、使用PID操纵云台
    4、反复循环进行以上步骤，当云台到达目标位置并且保持稳定后，进行射击，结束循环，从1、再次开始，累计射击10次
  */

  /*
    注意：机器人的开火方式可能需要自己设置
  */

  /*
    调试时需要做的是：
    1、修改满足射击条件的判断
  */
  const int shoot_total = 10;  //根据要求，射击进行10次
  int shoot_count = 0;         //当前已经完成的射击次数

  // 这是发送并且记录控制指令的环节，这个地方常用，而且很容易出问题，故而使用lambda表达式单独列出
  // 修改时，需要同时修改另外两个文件的对应函数
  auto send_command = [&gimbal, &plotter](
                        double yaw_target, double pitch_target, bool fire = false) -> void {
    //使用PID控制算法发送指令
    auto state = gimbal.state();  // 当前云台状态
    gimbal.send(true, fire, yaw_target, pitch_target);
    // 使用plotter绘制向云台发送的控制信息
    nlohmann::json data;
    data["yaw"] = yaw_target;
    data["pitch"] = pitch_target;
    
    plotter.plot(data);
  };

  std::cout << "started" << std::endl;
  while (!exiter.exit()) {
    // Your code start

    /************下面的代码与task_1相同************/
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
    nlohmann::json data;
    data["d_pitch"] = gimbal.state().pitch - pitch_target;
    data["d_yaw"] = gimbal.state().yaw - yaw_target;
    data["gimbal_pitch"] = gimbal.state().pitch;
    data["gimbal_yaw"] = gimbal.state().yaw;

    plotter.plot(data);

    if (
      abs(gimbal.state().pitch - pitch_target) <
        0.005 &&  //射击条件这里其实也不太清楚，目前限制为当前状态与目标状态的yaw与pitch
      abs(gimbal.state().yaw - yaw_target) < 0.005)  //相差在0.005rad之内，之后肯定需要调
    {
      if (shoot_count < shoot_total) {
        // 符合射击条件，发送射击指令
        pitch_target+=0.008;
        yaw_target-=0.005;
        send_command(yaw_target, pitch_target, true);
        send_command(yaw_target, pitch_target, false);
        // send_command(yaw_target, pitch_target, false);
        std::cout << "fire" << std::endl;
        shoot_count++;
        // 等待一段时间，然后重新开始循环
        std::this_thread::sleep_for(1500ms);
        continue;
      } else {
        // 射击次数达到上限，结束循环
        send_command(yaw_target, pitch_target, false);
        continue;
        //   break;
      }
    }
    send_command(yaw_target, pitch_target);

    // Your code end
  }

  return 0;
}