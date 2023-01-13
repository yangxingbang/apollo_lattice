/******************************************************************************
 * Copyright 2021 The DeepWay Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file truck_config_helper.cc
 */

#include "modules/planning/common/configs/truck_config_helper.h"

#include "cyber/common/file.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

TruckConfig TruckConfigHelper::truck_config_;
bool TruckConfigHelper::is_init_ = false;

TruckConfigHelper::TruckConfigHelper() {}

void TruckConfigHelper::Init() { Init(FLAGS_truck_config_path); }

void TruckConfigHelper::Init(const std::string& config_file) {
  TruckConfig params;
  AINFO << "load truck params";
  CHECK(cyber::common::GetProtoFromFile(config_file, &params))
      << "Unable to parse truck config file " << config_file;
  Init(params);
}

void TruckConfigHelper::Init(const TruckConfig &truck_params) {
  truck_config_ = truck_params;
  is_init_ = true;
}

const TruckConfig &TruckConfigHelper::GetConfig() {
  if (!is_init_) {
    Init();
  }
  return truck_config_;
}

} // namespace planning
} // namespace apollo