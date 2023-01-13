/******************************************************************************
 * Copyright 2021 The DeepWay Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file mini_overhang_path_optimizer.h
 **/

#pragma once

#include <unordered_map>

#include "modules/planning/tasks/optimizers/path_optimizer.h"

namespace apollo {
namespace planning {

class MiniOverhangPathOptimizer : public PathOptimizer {
 public:
  explicit MiniOverhangPathOptimizer(const TaskConfig& config);
  virtual ~MiniOverhangPathOptimizer() = default;

 private:
  common::Status Process(const SpeedData& speed_data,
                         const ReferenceLine& reference_line,
                         const common::TrajectoryPoint& init_point,
                         const bool path_reusable,
                         PathData* const final_path_data) override;

  void GraduallyDiscretization(const PathBoundary& path_boundary);

 private:
  std::vector<ReferencePoint> reference_points_;
  std::vector<std::pair<double, double>> l_boundary_;
  std::unordered_map<int, double> index_deltas_;
};

} // namesapce planning
} // namespace apollo