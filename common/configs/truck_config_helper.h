/******************************************************************************
 * Copyright 2021 The DeepWay Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file truck_config_helper.h
 */

#pragma once

#include <string>

#include "modules/planning/proto/truck_config.pb.h"
#include "cyber/common/macros.h"

namespace apollo {
namespace planning {

class TruckConfigHelper {
 public:
  static void Init();

  static void Init(const TruckConfig& config);

  static void Init(const std::string& config_file);

  static const TruckConfig& GetConfig();

 private:
  static TruckConfig truck_config_;
  static bool is_init_;
  DECLARE_SINGLETON(TruckConfigHelper);
};

} // namespace planning
} // namespace apollo