/******************************************************************************
 * Copyright 2021 The DeepWay Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file single_body.h
 **/

#pragma once

#include <array>
#include <vector>
#include <unordered_map>

#include "Eigen/Core"

#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/common/path/discretized_path.h"
#include "modules/planning/math/qpsolver/qp_solver.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

class MiniOverhangPathCarProblem : public QPSolver {
 public:
  MiniOverhangPathCarProblem(
      const std::vector<ReferencePoint>& reference_points, 
      const std::unordered_map<int, double>& index_deltas, 
      const MiniOverhangPathWeights& config, const int max_iter);
  virtual ~MiniOverhangPathCarProblem() = default;
  DiscretizedPath GetOptDiscretizedPath();
  
  void SetInitPointConstraint(const std::array<double, 4>& init_state) {
    init_state_ = init_state;
  }

  void SetEndPointConstraint(const std::array<double, 4>& end_state,
                             const bool is_end_state_constrainted) {
    end_state_ = end_state;
    is_end_point_constrainted_ = is_end_state_constrainted;
  }

  void SetLBoundary(const std::vector<std::pair<double, double>>& boundary) {
    l_boundary_ = boundary;
  }

  void SetCuiseSpeed(const double& speed) { v_cruise_ = speed; }

 private:
  MiniOverhangPathCarProblem() = default;

  void CalculateKernel(std::vector<c_float>* P_data, 
                       std::vector<c_int>* P_indices, 
                       std::vector<c_int>* P_indptr) override;

  void CalculateOffset(std::vector<c_float>* q) override;

  void CalculateAffineConstraint(
      std::vector<c_float>* A_data, std::vector<c_int>* A_indices, 
      std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds, 
      std::vector<c_float>* upper_bounds) override;

  void SetWarmStartX() override;

  void CalculateVehicleStateSpace(
      const double& kappa, Eigen::MatrixXd& sys,
      Eigen::MatrixXd& control, Eigen::MatrixXd& dis);

  void CalculateVehicleCorners(std::array<std::array<double, 3>, 4>& coefs);
  
  void CalculateVehicleCorners(
      const double& kappa, std::array<std::array<double, 3>, 4>& coefs);

  double denominator(const double kappa, const double& length) {
    return std::sqrt(length * length + 1.0 / kappa / kappa);
  }

 private:
  std::vector<ReferencePoint> reference_points_;
  std::unordered_map<int, double> index_deltas_;
  MiniOverhangPathWeights config_;
  double delta_s_;
  std::array<double, 4> init_state_;
  std::array<double, 4> end_state_;
  std::vector<std::pair<double, double>> l_boundary_;
  double v_cruise_;
  bool is_end_point_constrainted_;
};

} // namesapce planning
} // namespace apollo