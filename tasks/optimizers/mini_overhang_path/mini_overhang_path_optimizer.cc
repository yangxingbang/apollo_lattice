/******************************************************************************
 * Copyright 2021 The DeepWay Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file mini_overhang_path_optimizer.h
 **/

#include "modules/planning/tasks/optimizers/mini_overhang_path/mini_overhang_path_optimizer.h"

#include <string>
#include <cmath>

#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/tasks/optimizers/mini_overhang_path/mini_overhang_path_truck_problem.h"
#include "modules/planning/tasks/optimizers/mini_overhang_path/mini_overhang_path_car_problem.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/common/tractor_trailer_model.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

MiniOverhangPathOptimizer::MiniOverhangPathOptimizer(const TaskConfig& config) 
    : PathOptimizer(config) {
  CHECK(config_.has_mini_overhang_path_config());
}

common::Status MiniOverhangPathOptimizer::Process(
    const SpeedData& speed_data, const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point, const bool path_reusable,
    PathData* const final_path_data) {
  // skip piecewise_jerk_path_optimizer if reused path
  if (FLAGS_enable_skip_path_tasks && path_reusable) {
    return Status::OK();
  }
  ADEBUG << "Plan at the starting point: x = " << init_point.path_point().x()
         << ", y = " << init_point.path_point().y()
         << ", and angle = " << init_point.path_point().theta();
  
  std::array<double, 4> init_state;
  std::array<double, 4> end_state{0.0, 0.0, 0.0, 0.0};
  bool is_end_state_constrainted = false;
  const common::PathPoint& init_path_point = init_point.path_point();
  auto init_ref_point = reference_line.GetReferencePoint(init_path_point.x(), init_path_point.y());
  auto init_frent_point = reference_line.GetFrenetPoint(init_path_point);
  init_state[0] = common::math::AngleDiff(init_ref_point.heading(), init_path_point.theta());
  init_state[1] = init_frent_point.l();
  init_state[2] = common::math::AngleDiff(init_path_point.theta(), init_path_point.theta_trailer());
  init_state[3] = init_path_point.kappa();
  ADEBUG << "Init states: tractor_theta_error = " << init_state[0] << ", l = "
         << init_state[1] << ", trailer_lambda = " << init_state[2]
         << ", kappa = " << init_state[3];
  ADEBUG << "ref_point_theta = " << init_ref_point.heading()
         << ", init_path_point_theta = " << init_path_point.theta()
         << ", init_path_point_theta_trailer = " << init_path_point.theta_trailer();

  // Choose lane_change_path_config for lane-change cases
  // Otherwise, choose default_path_config for normal path planning
  auto mini_overhang_path_config =
      reference_line_info_->IsChangeLanePath()
          ? config_.mini_overhang_path_config().lane_change_path_config()
          : config_.mini_overhang_path_config().default_path_config();
  mini_overhang_path_config.set_phi_weight(
    std::fmax(1.0, init_point.v()) * mini_overhang_path_config.phi_weight());
  mini_overhang_path_config.set_kappa_weight(
    std::fmax(1.0, init_point.v()) * mini_overhang_path_config.kappa_weight());

  int max_iter = 50000;
  const auto& path_boundaries =
      reference_line_info_->GetCandidatePathBoundaries();
  ADEBUG << "There are " << path_boundaries.size() << " path boundaries.";
  std::vector<PathData> candidate_path_data;
  for (const auto& path_boundary : path_boundaries) {
    if (!FLAGS_enable_force_pull_over_open_space_parking_test) {
      // pull over scenario
      // set end lateral to be at the desired pull over destination
      const auto& pull_over_status =
          PlanningContext::Instance()->planning_status().pull_over();
      if (pull_over_status.has_position() &&
          pull_over_status.position().has_x() &&
          pull_over_status.position().has_y() &&
          path_boundary.label().find("pullover") != std::string::npos) {
        common::SLPoint pull_over_sl;
        reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
        end_state[0] = 0.0;
        end_state[1] = pull_over_sl.l();
        end_state[2] = 0.0;
        is_end_state_constrainted = true;
      }
    }
    // if the path_boundary is normal, it is possible to have less than 2 points
    // skip path boundary of this kind
    if (path_boundary.label().find("regular") != std::string::npos &&
        path_boundary.boundary().size() < 2) {
      continue;
    }
    CHECK_GT(path_boundary.boundary().size(), 1);
    
    GraduallyDiscretization(path_boundary);

    for (size_t i = 0; i < l_boundary_.size(); ++i) {
      ADEBUG << "For i = " << i << ", l_boundary_: [" << l_boundary_[i].first
             << ", " << l_boundary_[i].second << "]";
    }

    MiniOverhangPathTruckProblem mini_overhang_truck_problem(
        reference_points_, index_deltas_, mini_overhang_path_config, max_iter);
    mini_overhang_truck_problem.SetInitPointConstraint(init_state);
    mini_overhang_truck_problem.SetEndPointConstraint(end_state, is_end_state_constrainted);
    mini_overhang_truck_problem.SetLBoundary(l_boundary_);
    mini_overhang_truck_problem.SetCuiseSpeed(reference_line_info_->GetCruiseSpeed());
    bool successed = mini_overhang_truck_problem.Optimize();

    ADEBUG << "Solving mini_overhang_truck_problem succeed: " << successed;

    // MiniOverhangPathCarProblem mini_overhang_car_problem(
    //     reference_points_, index_deltas_, mini_overhang_path_config, max_iter);
    // mini_overhang_car_problem.SetInitPointConstraint(init_state);
    // mini_overhang_car_problem.SetEndPointConstraint(end_state, is_end_state_constrainted);
    // mini_overhang_car_problem.SetLBoundary(l_boundary_);
    // mini_overhang_car_problem.SetCuiseSpeed(reference_line_info_->GetCruiseSpeed());
    // bool successed = mini_overhang_car_problem.Optimize();

    if (successed) {
      DiscretizedPath discretized_path = mini_overhang_truck_problem.GetOptDiscretizedPath();
      // DiscretizedPath discretized_path = mini_overhang_car_problem.GetOptDiscretizedPath();
      PathData path_data = *final_path_data;
      path_data.SetReferenceLine(&reference_line);
      path_data.SetDiscretizedPath(discretized_path);
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
      candidate_path_data.push_back(std::move(path_data));
    }
  }
  if (candidate_path_data.empty()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Path Optimizer failed to generate path");
  }
  reference_line_info_->SetCandidatePathData(std::move(candidate_path_data));
  return Status::OK();
}

void MiniOverhangPathOptimizer::GraduallyDiscretization(const PathBoundary& path_boundary) {
  reference_points_.clear();
  l_boundary_.clear();
  index_deltas_.clear();
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  const std::vector<std::pair<double, double>>& l_boundary = path_boundary.boundary();
  const double delta_s = path_boundary.delta_s();
  const double start_s = path_boundary.start_s();
  for (size_t i = 0, splice_index = 1; i < path_boundary.boundary().size();) {
    double s = start_s + i * delta_s;
    ReferencePoint ref_point = reference_line.GetNearestReferencePoint(s);
    if (i == 0) {
      ref_point = reference_line.GetNearestReferencePoint(s + 0.05);
    }
    reference_points_.emplace_back(std::move(ref_point));
    l_boundary_.emplace_back(std::move(l_boundary[i]));
    index_deltas_.insert(std::pair<int, double>(reference_points_.size() - 1, splice_index * delta_s));
    if (s > start_s + splice_index * FLAGS_gradually_discreted_splice) {
      ++splice_index;
    }
    i += splice_index;
  }
}

} // namespace planning
} // namespace apollo