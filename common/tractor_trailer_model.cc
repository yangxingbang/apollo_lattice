/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/common/tractor_trailer_model.h"

#include "modules/planning/common/frame.h"
#include "modules/common/status/status.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/common/trailer_state_provider.h"
#include "modules/planning/common/configs/truck_config_helper.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleState;

double TractorTrailerModel::prev_trailer_yaw_ = 0.0;

void TractorTrailerModel::UpdateTrailerYawAngle(
    const VehicleState& curr_tractor_state) {

  const uint32_t is_tractor_go_straight_threshold = 40;

  double curr_trailer_yaw = curr_tractor_state.heading();

  const auto* latest_frame = FrameHistory::Instance()->Latest();
  if (!latest_frame) {
    AINFO << "Lastest frame is empty";
    prev_trailer_yaw_ = curr_trailer_yaw;
    return;
  }

  const int latest_frame_num = latest_frame->SequenceNum();
  // Suppose trailer yaw is equal to tractor yaw at the starting point.
  if (latest_frame_num < FLAGS_max_frame_history_num) {
    AINFO << "Current frame sequanece num. is less than " << 
        FLAGS_max_frame_history_num;
    prev_trailer_yaw_ = curr_trailer_yaw;
    return;
  }

  uint32_t tractor_go_straight_cycle = 0;
  for (int i = 0; i < FLAGS_max_frame_history_num; ++i) {
    const auto* temp_frame = FrameHistory::Instance()->Find(latest_frame_num - i);
    if (!temp_frame) {
      AINFO << "temp_frame is empty";
      prev_trailer_yaw_ = curr_trailer_yaw;
      return;
    }
    if (temp_frame->vehicle_state().linear_velocity() > 0.1 && 
        std::fabs(temp_frame->vehicle_state().kappa()) < 0.01) {
      tractor_go_straight_cycle++;
    }
  }
  if (tractor_go_straight_cycle > is_tractor_go_straight_threshold) {
    AINFO << "Tractor goes straight";
    prev_trailer_yaw_ = curr_trailer_yaw;
    return;
  }

  AINFO << "Get trailer yaw angle by trailer kinematic model";
  const auto& prev_tractor_state = latest_frame->vehicle_state();
  curr_trailer_yaw = TrailerStateProvider::GetTrailerTheta(
      prev_tractor_state, curr_tractor_state, prev_trailer_yaw_);
  prev_trailer_yaw_ = curr_trailer_yaw;

  return;
}

common::TrajectoryPoint TractorTrailerModel::CalculateTrailerCoordinate(
    const common::TrajectoryPoint& tractor_point) {
  common::TrajectoryPoint trailer_point = tractor_point;
  const double theta_tractor = tractor_point.path_point().theta();
  const double theta_trailer = tractor_point.path_point().theta_trailer();
  const double head_base_to_hinge = 
      TruckConfigHelper::GetConfig().trailer_param().head_base_to_hinge();
  const double length_trailer = 
      TruckConfigHelper::GetConfig().trailer_param().wheel_base() +
      TruckConfigHelper::GetConfig().trailer_param().rear_overhang();

  double x_hinge = tractor_point.path_point().x() - 
      head_base_to_hinge * std::cos(theta_tractor);
  double y_hinge = tractor_point.path_point().y() - 
      head_base_to_hinge * std::sin(theta_tractor);
  
  double x_trailer = x_hinge - length_trailer * std::cos(theta_trailer);
  double y_trailer = y_hinge - length_trailer * std::sin(theta_trailer);

  auto path_point_trailer = trailer_point.mutable_path_point();
  path_point_trailer->set_x(x_trailer);
  path_point_trailer->set_y(y_trailer);
  return trailer_point;
}

}  // namespace planning
}  // namespace apollo
