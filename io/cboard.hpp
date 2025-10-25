#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include "io/command.hpp"

namespace io
{

// 模式定义（其实 aimer 不会用到）
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};

// 射击模式定义
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};

// 仅保留向后兼容所需的常量
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

}  // namespace io

#endif  // IO__CBOARD_HPP
