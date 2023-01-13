/******************************************************************************
 * Copyright 2021 The DeepWay Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"

namespace apollo {
namespace planning {

class TrailerStateProvider {
 public:
  static double GetTrailerTheta(
      const common::VehicleState& prev_tractor_state,
      const common::VehicleState& curr_tractor_state,
      const double prev_trailer_yaw);
};

}  // namespace planning
}  // namespace apollo