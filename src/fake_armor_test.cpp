// fake_target_test.cpp
// 独立测试 Target 的假装甲板模拟器 (C++)
// 用法: ./fake_target_test [omega(rad/s)] [radius(m)] [center_x] [center_y] [dt]
// 例: ./fake_target_test 4.0 0.2 2.0 0.0 0.033

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <thread>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

auto_aim::Armor make_fake_armor(
  double angle, double center_x, double center_y, double center_z, double radius,
  std::mt19937 & rng)
{
  using namespace auto_aim;

  // -----------------------------
  //  构造装甲板基本形状
  // -----------------------------
  std::vector<cv::Point2f> keypoints = {
    cv::Point2f(-50, -25), cv::Point2f(50, -25), cv::Point2f(50, 25), cv::Point2f(-50, 25)};
  cv::Rect box(0, 0, 100, 50);

  Armor armor(0, 1, 1.0f, box, keypoints);  // color=blue, id=1, conf=1.0

  // -----------------------------
  //  理想圆周轨迹 (绕 z 轴)
  // -----------------------------
  double ax = center_x - radius * std::cos(angle);
  double ay = center_y - radius * std::sin(angle);
  double az = center_z;
  armor.xyz_in_world = Eigen::Vector3d(ax, ay, az);

  // -----------------------------
  //  真实装甲姿态（切线方向）
  // 装甲面法线始终朝向切线方向
  // -----------------------------
  double armor_yaw = angle + M_PI / 2.0;        // 切向方向
  if (armor_yaw > M_PI) armor_yaw -= 2 * M_PI;  // wrap 到 [-π, π]

  // -----------------------------
  //  添加测量噪声
  // -----------------------------
  std::normal_distribution<double> noise_yaw(0.0, 0.005);    // ≈0.3°
  std::normal_distribution<double> noise_pitch(0.0, 0.003);  // ≈0.17°
  std::normal_distribution<double> noise_dist(0.0, 0.01);    // 1cm

  double noisy_yaw = std::atan2(ay, ax) + noise_yaw(rng);
  double noisy_pitch = std::atan2(az, std::sqrt(ax * ax + ay * ay)) + noise_pitch(rng);
  double noisy_dist = std::sqrt(ax * ax + ay * ay + az * az) + noise_dist(rng);

  // -----------------------------
  //  写入 Armor 对象
  // -----------------------------
  armor.ypd_in_world = Eigen::Vector3d(noisy_yaw, noisy_pitch, noisy_dist);
  armor.ypr_in_world = Eigen::Vector3d(armor_yaw, 0.0, 0.0);
  armor.confidence = 1.0;

  return armor;
}

int main(int argc, char ** argv)
{
  tools::Plotter plotter;

  double omega = 4.0;   // 角速度 rad/s
  double radius = 0.4;  // 半径 m
  double center_x = 2.0;
  double center_y = 0.0;
  double center_z = 0.0;
  double dt = 0.2;  // 秒
  int max_steps = 2000;

  if (argc > 1) omega = std::stod(argv[1]);
  if (argc > 2) radius = std::stod(argv[2]);
  if (argc > 3) center_x = std::stod(argv[3]);
  if (argc > 4) center_y = std::stod(argv[4]);
  if (argc > 5) dt = std::stod(argv[5]);

  std::cout << "[FakeTargetTest] omega=" << omega << " rad/s, radius=" << radius << " m, center=("
            << center_x << "," << center_y << "," << center_z << "), dt=" << dt << "s\n";

  std::mt19937 rng(12345);
  double angle = 0.0;
  auto t_now = std::chrono::steady_clock::now();

  // 第一个假装甲板
  auto armor = make_fake_armor(angle, center_x, center_y, center_z, radius, rng);

  Eigen::VectorXd x0(11);
  x0 << center_x - radius, 0.0, center_y, 0.0, center_z, 0.0, 0.0, omega, radius, 0.0, 0.0;
  auto_aim::Target target(armor, t_now, radius, 4, x0);
  std::cout << "[FakeTargetTest] Target created.\n";

  for (int step = 0; step < max_steps; ++step) {
    angle += omega * dt;
    t_now += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(dt));

    armor = make_fake_armor(angle, center_x, center_y, center_z, radius, rng);

    nlohmann::json data;
    data["armor_x"] = armor.xyz_in_world.x();
    data["armor_y"] = armor.xyz_in_world.y();
    data["armor_z"] = armor.xyz_in_world.z();
    plotter.plot(data);

    target.predict(t_now);
    target.update(armor);

    Eigen::VectorXd x = target.ekf_x();
    std::cout << "[step " << step << "] t=" << step * dt << "s, x=" << x.transpose() << "\n";

    if (target.diverged()) {
      std::cout << "=> Target diverged at step " << step << "\n";
      break;
    }
    if (target.convergened()) {
      nlohmann::json data;
      data["real_omega"] = omega;
      data["predict_omega"] = target.ekf_x()[7];
      data["real_r"] = radius;
      data["predict_r"] = x[8];
      plotter.plot(data);
      std::cout << "=> Target converged at step " << step << ", w=" << x[7] << ", a=" << x[6]
                << ", r=" << x[8] << "\n";
      // break; // 如希望收敛后退出可取消注释
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
  }

  std::cout << "[FakeTargetTest] finished\n";
  return 0;
}
