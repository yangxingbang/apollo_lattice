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

#pragma once

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace planning {

class TractorTrailerModel {
 public:
  static void UpdateTrailerYawAngle(
      const common::VehicleState& curr_tractor_state);
  
  static double GetTrailerYawAngle() {
    return prev_trailer_yaw_;
  }

  static double GetCurrTrailerYaw(
      const common::VehicleState& prev_tractor_state,
      const common::VehicleState& curr_tractor_state);
  
  static common::TrajectoryPoint CalculateTrailerCoordinate(
      const common::TrajectoryPoint& tractor_point);

 private:
  static double prev_trailer_yaw_;
};

}  // namespace planning
}  // namespace apollo
