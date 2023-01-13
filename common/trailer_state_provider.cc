/******************************************************************************
 * Copyright 2021 The DeepWay Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/planning/common/trailer_state_provider.h"

#include "modules/planning/common/configs/truck_config_helper.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace planning {

double TrailerStateProvider::GetTrailerTheta(
    const common::VehicleState& prev_tractor_state,
    const common::VehicleState& curr_tractor_state,
    const double prev_trailer_yaw) {
    const auto& trailer_param = TruckConfigHelper::GetConfig().trailer_param();

  const double trailer_wheelbase = trailer_param.wheel_base();
  const double dist_tractor_rear_center_to_hinge = trailer_param.head_base_to_hinge();

  double curr_trailer_yaw = prev_trailer_yaw;

  double prev_tractor_x = prev_tractor_state.x();
  double prev_tractor_y = prev_tractor_state.y();
  double prev_tractor_yaw = prev_tractor_state.heading();
  double curr_tractor_x = curr_tractor_state.x();
  double curr_tractor_y = curr_tractor_state.y();
  double curr_tractor_yaw = curr_tractor_state.heading();

  double delta_tractor_yaw = common::math::AngleDiff(prev_tractor_yaw,
      curr_tractor_yaw);
  double delta_length = std::sqrt(
      common::math::Square(curr_tractor_x - prev_tractor_x) + 
      common::math::Square(curr_tractor_y - prev_tractor_y));
   double delta_trailer_yaw = 0.0;

  if (delta_tractor_yaw <= -0.01 || delta_tractor_yaw >= 0.01) {
    // 间隔yaw角太大,则需要分段进⾏计算, 假定⻋头按圆弧运动 
    // arc_length: 圆弧总⻓度 
    // step_arc_length: 每段圆弧⻓ 
    // min_num: 设定的最⼩分段数
    const int min_num = 5;
    double arc_length = 
        delta_length / 2.0 / std::sin(delta_tractor_yaw / 2.0) *
        delta_tractor_yaw;
    double step_arc_length = 0.1;
    int step_num = std::floor(arc_length / step_arc_length) > min_num ?
        std::floor(arc_length / step_arc_length) : min_num;
    step_arc_length = arc_length / step_num;
    double step_delta_yaw = delta_tractor_yaw / step_num;

    for (int i = 0; i < step_num; ++i) {
      double current_step_yaw = prev_tractor_yaw + i * step_delta_yaw;
      delta_trailer_yaw = 
          step_arc_length / trailer_wheelbase * std::sin(current_step_yaw - 
          curr_trailer_yaw) - dist_tractor_rear_center_to_hinge *
          step_delta_yaw / trailer_wheelbase * std::cos(current_step_yaw - 
          curr_trailer_yaw);
      curr_trailer_yaw += delta_trailer_yaw;
      curr_trailer_yaw = common::math::NormalizeAngle(curr_trailer_yaw);          
    }
  } else {
    delta_trailer_yaw = 
        delta_length / trailer_wheelbase * std::sin(prev_tractor_yaw - 
        curr_trailer_yaw) - dist_tractor_rear_center_to_hinge * 
        delta_trailer_yaw / trailer_wheelbase * std::cos(prev_tractor_yaw - 
        curr_trailer_yaw);
    curr_trailer_yaw += delta_trailer_yaw;
    curr_trailer_yaw = common::math::NormalizeAngle(curr_trailer_yaw);    
  }

  return curr_trailer_yaw;
}

}  // namespace planning
}  // namespace apollo