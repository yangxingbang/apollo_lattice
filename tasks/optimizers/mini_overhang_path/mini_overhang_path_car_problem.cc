/******************************************************************************
 * Copyright 2021 The DeepWay Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file single_body.cc
 **/

#include "modules/planning/tasks/optimizers/mini_overhang_path/mini_overhang_path_car_problem.h"

#include <string>

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/configs/truck_config_helper.h"
#include "modules/common/configs/vehicle_config_helper.h" 
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace planning {

MiniOverhangPathCarProblem::MiniOverhangPathCarProblem(
    const std::vector<ReferencePoint>& reference_points, 
    const std::unordered_map<int, double>& index_deltas, 
    const MiniOverhangPathWeights& config, const int max_iter)
    : QPSolver(max_iter), reference_points_(reference_points)
    , index_deltas_(index_deltas), config_(config), delta_s_(0.5) {
  kernel_dim_ = 6 * reference_points_.size() - 1;
  is_end_point_constrainted_ = false;
}

DiscretizedPath MiniOverhangPathCarProblem::GetOptDiscretizedPath() {
  size_t num_of_knots = reference_points_.size();
  std::vector<common::PathPoint> ret(num_of_knots);
  common::PathPoint point;
  double accumulated_s = 0.0;
  double x_last, y_last = 0.0;
  for (size_t i = 0; i < num_of_knots; ++i) {
    double ref_x = reference_points_[i].x();
    double ref_y = reference_points_[i].y();
    double ref_theta = reference_points_[i].heading();
    double x = ref_x - solution_[2 * i + 1] * std::sin(ref_theta);
    double y = ref_y + solution_[2 * i + 1] * std::cos(ref_theta);
    double theta = common::math::NormalizeAngle(solution_[2 * i] + ref_theta);
    double kappa = 0.0;
    if (i == 0) {
      kappa = init_state_[3];
    } else if (i < num_of_knots - 1) {
      kappa = solution_[2 * num_of_knots + i];
    } else {
      kappa = solution_[3 * num_of_knots - 2];
    }
    double dkappa = 0.0;
    double delta_s = index_deltas_.find(i)->second;
    if (i == 0) {
      dkappa = (solution_[2 * num_of_knots + i + 1] - kappa) / delta_s;
    } else if (i == num_of_knots - 1) {
      dkappa = (kappa - solution_[2 * num_of_knots + i - 2]) / delta_s;
    } else {
      dkappa = (kappa - solution_[2 * num_of_knots + i - 1]) / delta_s;
    }
    if (i > 0.5) {
      accumulated_s += std::hypot(x - x_last, y - y_last);
    }
    x_last = x; y_last = y;
    point.set_x(x); point.set_y(y); point.set_z(0.0);
    point.set_theta(theta); point.set_kappa(kappa);
    point.set_s(accumulated_s); point.set_dkappa(dkappa);
    ret[i] = point;
  }
  return DiscretizedPath(ret);
}

void MiniOverhangPathCarProblem::CalculateKernel(
    std::vector<c_float>* P_data, std::vector<c_int>* P_indices, 
    std::vector<c_int>* P_indptr) {
  P_data->clear(); P_indices->clear(); P_indptr->clear();
  size_t num = reference_points_.size();
  size_t count = 0;
  P_indptr->push_back(count);
  for (size_t i = 0; i < num; ++i) {
    P_data->push_back(config_.phi_weight());
    P_indices->push_back(2 * i);
    P_indptr->push_back(++count);
    P_data->push_back(config_.l_weight());
    P_indices->push_back(2 * i + 1);
    P_indptr->push_back(++count);
  }
  for (size_t i = 0; i < num - 1; ++i) {
    if (i == 0) {
      P_data->push_back(config_.kappa_weight() + config_.dkappa_weight());
      P_indices->push_back(2 * num + i);
      P_data->push_back(- config_.dkappa_weight());
      P_indices->push_back(2 * num + i + 1);
      count += 2;
      P_indptr->push_back(count);
    } else if (i == num - 2) {
      P_data->push_back(- config_.dkappa_weight());
      P_indices->push_back(2 * num + i - 1);
      P_data->push_back(config_.kappa_weight() + config_.dkappa_weight());
      P_indices->push_back(2 * num + i);
      count += 2;
      P_indptr->push_back(count);
    } else {
      P_data->push_back(- config_.dkappa_weight());
      P_indices->push_back(2 * num + i - 1);
      P_data->push_back(config_.kappa_weight() + 2 * config_.dkappa_weight());
      P_indices->push_back(2 * num + i);
      P_data->push_back(- config_.dkappa_weight());
      P_indices->push_back(2 * num + i + 1);
      count += 3;
      P_indptr->push_back(count);
    }
  }
  for (size_t i = 0; i < num; ++i) {
    P_data->push_back(config_.slack_wheelbase_tractor_weight());
    P_indices->push_back(3 * num - 1 + 3 * i);
    P_indptr->push_back(++count);
    P_data->push_back(config_.slack_front_overhang_tractor_weight());
    P_indices->push_back(3 * num - 1 + 3 * i + 1);
    P_indptr->push_back(++count);
    P_data->push_back(config_.slack_rear_overhang_tractor_weight());
    P_indices->push_back(3 * num - 1 + 3 * i + 2);
    P_indptr->push_back(++count);
  }
}

void MiniOverhangPathCarProblem::CalculateOffset(std::vector<c_float>* q) {
  q->resize(kernel_dim_, 0.0);
}

void MiniOverhangPathCarProblem::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices, 
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds, 
    std::vector<c_float>* upper_bounds) {
  const int num_of_knots = reference_points_.size();

  // The number of optimization variables, number of columns of the constraint matrix
  const int num_of_variables = 6 * num_of_knots - 1;
  // The number of constraints, the number of rows of the constraint matrix
  const int num_of_constraints = 15 * num_of_knots - 4;
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);
  
  // num_of_variables: number of columns of the constraint matrix, outer loop
  // inner loop: rows of the constraint matrix(variables)
  // c_float: elements of constraint matri
  // c_int: rows of the constraint matrix
  std::vector<std::vector<std::pair<c_int, c_float>>> variables(
      num_of_variables);

  // row of constraint
  int constraint_index = 0;


  variables[0].emplace_back(constraint_index, -1.0);
  lower_bounds->at(constraint_index) = -init_state_[0];
  upper_bounds->at(constraint_index) = -init_state_[0];
  ++constraint_index;

  variables[1].emplace_back(constraint_index, -1.0);
  lower_bounds->at(constraint_index) = -init_state_[1];
  upper_bounds->at(constraint_index) = -init_state_[1];
  ++constraint_index;

  variables[2 * num_of_knots].emplace_back(constraint_index, -1.0);
  lower_bounds->at(constraint_index) = -init_state_[3];
  upper_bounds->at(constraint_index) = -init_state_[3];
  ++constraint_index;

  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double kappa_max =
      std::tan(veh_param.max_steer_angle() / veh_param.steer_ratio()) /
      veh_param.wheel_base();
  const double dkappa_max =
      std::tan(veh_param.max_steer_angle_rate() / veh_param.steer_ratio()) /
      veh_param.wheel_base();

  // equality constraints of vehicle kinematics
  Eigen::MatrixXd sys(3, 3), control(3, 1), dis(3, 1);
  for (int i = 1; i < num_of_knots; ++i) {
    double kappa = reference_points_[i].kappa();
    delta_s_ = index_deltas_.find(i)->second;
    CalculateVehicleStateSpace(kappa, sys, control, dis);

    variables[2 * i - 2].emplace_back(constraint_index, sys(0, 0));
    variables[2 * i - 1].emplace_back(constraint_index, sys(0, 1));
    variables[2 * i].emplace_back(constraint_index, -1.0);
    variables[2 * num_of_knots + i - 1].emplace_back(constraint_index, control(0, 0));
    lower_bounds->at(constraint_index) = -dis(0, 0);
    upper_bounds->at(constraint_index) = -dis(0, 0);
    ++constraint_index;
    
    variables[2 * i - 2].emplace_back(constraint_index, sys(1, 0));
    variables[2 * i - 1].emplace_back(constraint_index, sys(1, 1));
    variables[2 * i + 1].emplace_back(constraint_index, -1.0);
    lower_bounds->at(constraint_index) = -dis(1, 0);
    upper_bounds->at(constraint_index) = -dis(1, 0);
    ++constraint_index;
  }

  // Upper and lower boundary constraints of e_ph and e_y
  for (int i = 1; i < num_of_knots; ++i) {
    if (i == num_of_knots - 1 && is_end_point_constrainted_) {
      variables[2 * i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) = end_state_[0];
      upper_bounds->at(constraint_index) = end_state_[0];
      ++constraint_index;
    
      variables[2 * i + 1].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) = end_state_[1];
      upper_bounds->at(constraint_index) = end_state_[1];
      ++constraint_index;
    } else {
      variables[2 * i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) = -FLAGS_max_heading_error;
      upper_bounds->at(constraint_index) = FLAGS_max_heading_error;
      ++constraint_index;
    
      variables[2 * i + 1].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) = l_boundary_[i].first;
      upper_bounds->at(constraint_index) = l_boundary_[i].second;
      ++constraint_index;
    }
  }
  // Upper and lower boundary constraints of kappa
  for (int i = 0; i < num_of_knots - 1; ++i) {
    variables[2 * num_of_knots + i].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) = -kappa_max;
    upper_bounds->at(constraint_index) = kappa_max;
    ++constraint_index;
  }

  // Upper and lower boundary constraints of dkappa
  for (int i = 1; i < num_of_knots - 1; ++i) {
    variables[2 * num_of_knots + i - 1].emplace_back(constraint_index, -1.0);
    variables[2 * num_of_knots + i].emplace_back(constraint_index, 1.0);
    delta_s_ = index_deltas_.find(i)->second;
    lower_bounds->at(constraint_index) = -dkappa_max * delta_s_ / (v_cruise_ + 1e-5);
    upper_bounds->at(constraint_index) = dkappa_max * delta_s_ / (v_cruise_ + 1e-5);
    ++constraint_index;
  }

  // Upper and lower boundary constraints of slack factor
  for (int i = 0; i < num_of_knots; ++ i) {
    variables[3 * num_of_knots - 1 + 3 * i].emplace_back(constraint_index, 1);
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 1e10;
    ++constraint_index;

    variables[3 * num_of_knots - 1 + 3 * i + 1].emplace_back(constraint_index, 1);
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 1e10;
    ++constraint_index;

    variables[3 * num_of_knots - 1 + 3 * i + 2].emplace_back(constraint_index, 1);
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 1e10;
    ++constraint_index;
  }

  // Position constraint of tractor corner
  std::array<std::array<double, 3>, 4> coefs;
  for (int i = 0; i < num_of_knots; ++i) {
    double kappa = reference_points_[i].kappa();
    if (std::fabs(kappa) > 1e-10) {
      CalculateVehicleCorners(kappa, coefs);
    } else {
      CalculateVehicleCorners(coefs);
    }
    variables[2 * i].emplace_back(constraint_index, coefs[1][0]);
    variables[2 * i + 1].emplace_back(constraint_index, coefs[1][1]);
    variables[3 * num_of_knots - 1 + 3 * i].emplace_back(constraint_index, 1);
    lower_bounds->at(constraint_index) = l_boundary_[i].first;
    upper_bounds->at(constraint_index) = 1e10;
    ++constraint_index;

    variables[2 * i].emplace_back(constraint_index, coefs[1][0]);
    variables[2 * i + 1].emplace_back(constraint_index, coefs[1][1]);
    variables[3 * num_of_knots - 1 + 3 * i].emplace_back(constraint_index, -1);
    lower_bounds->at(constraint_index) = -1e10;
    upper_bounds->at(constraint_index) = l_boundary_[i].second;
    ++constraint_index;

    variables[2 * i].emplace_back(constraint_index, coefs[0][0]);
    variables[2 * i + 1].emplace_back(constraint_index, coefs[0][1]);
    variables[3 * num_of_knots - 1 + 3 * i + 1].emplace_back(constraint_index, 1);
    lower_bounds->at(constraint_index) = l_boundary_[i].first;
    upper_bounds->at(constraint_index) = 1e10;
    ++constraint_index;

    variables[2 * i].emplace_back(constraint_index, coefs[0][0]);
    variables[2 * i + 1].emplace_back(constraint_index, coefs[0][1]);
    variables[3 * num_of_knots - 1 + 3 * i + 1].emplace_back(constraint_index, -1);
    lower_bounds->at(constraint_index) = -1e10;
    upper_bounds->at(constraint_index) = l_boundary_[i].second;
    ++constraint_index;

    variables[2 * i].emplace_back(constraint_index, coefs[3][0]);
    variables[2 * i + 1].emplace_back(constraint_index, coefs[3][1]);
    variables[3 * num_of_knots - 1 + 3 * i + 2].emplace_back(constraint_index, 1);
    lower_bounds->at(constraint_index) = l_boundary_[i].first;
    upper_bounds->at(constraint_index) = 1e10;
    ++constraint_index;

    variables[2 * i].emplace_back(constraint_index, coefs[3][0]);
    variables[2 * i + 1].emplace_back(constraint_index, coefs[3][1]);
    variables[3 * num_of_knots - 1 + 3 * i + 2].emplace_back(constraint_index, -1);
    lower_bounds->at(constraint_index) = -1e10;
    upper_bounds->at(constraint_index) = l_boundary_[i].second;
    ++constraint_index;
  }

  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    A_indptr->push_back(ind_p);
    for (const auto& variable_nz : variables[i]) {
      A_data->push_back(variable_nz.second);
      A_indices->push_back(variable_nz.first);
      ++ind_p;
    }
  }
  A_indptr->push_back(ind_p);
}

void MiniOverhangPathCarProblem::SetWarmStartX() {
  start_x_.resize(kernel_dim_, 0.0);
}

void MiniOverhangPathCarProblem::CalculateVehicleStateSpace(
    const double& kappa, Eigen::MatrixXd& sys,
    Eigen::MatrixXd& control, Eigen::MatrixXd& dis) {
  const auto& tractor_param = TruckConfigHelper::GetConfig().tractor_param();
  sys(0, 0) = 1; sys(0, 1) = -delta_s_ * kappa * kappa;
  sys(1, 0) = delta_s_; sys(1, 1) = 1;
  control(0, 0) = delta_s_ / std::pow(std::cos(std::atan(tractor_param.wheel_base() * kappa)), 2);
  control(1, 0) = 0.0;
  dis(0, 0) = -control(0, 0) * kappa;
  dis(1, 0) = 0.0;
}

void MiniOverhangPathCarProblem::CalculateVehicleCorners(
    std::array<std::array<double, 3>, 4>& coefs) {
  const auto& tractor_param = TruckConfigHelper::GetConfig().tractor_param();
  const double wheel_base = tractor_param.wheel_base();
  const double front_overhang = tractor_param.front_overhang();
  const double rear_overhang = tractor_param.rear_overhang();

  coefs[0][0] = wheel_base + front_overhang;
  coefs[0][1] = 1.0;
  coefs[0][2] = 0.0;

  coefs[1][0] = wheel_base;
  coefs[1][1] = 1.0;
  coefs[1][2] = 0.0;

  coefs[2][0] = 0.0;
  coefs[2][1] = 1.0;
  coefs[2][2] = 0.0;

  coefs[3][0] = -rear_overhang;
  coefs[3][1] = 1.0;
  coefs[3][2] = 0.0;
}

void MiniOverhangPathCarProblem::CalculateVehicleCorners(
    const double& kappa, std::array<std::array<double, 3>, 4>& coefs) {
  const auto& tractor_param = TruckConfigHelper::GetConfig().tractor_param();
  std::array<double, 4> length;
  const double front_overhang = tractor_param.front_overhang();
  const double wheel_base = tractor_param.wheel_base();
  const double rear_overhang = tractor_param.rear_overhang();

  length[0] = front_overhang + wheel_base;
  length[1] = wheel_base;
  length[2] = 0.0;
  length[3] = -rear_overhang;

  if (kappa > 0) {
    for (int i = 0; i < 4; ++i) {
      coefs[i][0] = length[i] / (denominator(kappa, length[i]) * kappa);
      coefs[i][1] = 1.0 / (denominator(kappa, length[i]) * kappa);
      coefs[i][2] = 1.0 / kappa - denominator(kappa, length[i]);
    }
  } else {
    for (int i = 0; i < 4; ++i) {
      coefs[i][0] = length[i] / (denominator(std::fabs(kappa), length[i]) * std::fabs(kappa));
      coefs[i][1] = 1.0 / (denominator(std::fabs(kappa), length[i]) * std::fabs(kappa));
      coefs[i][2] = denominator(std::fabs(kappa), length[i]) - 1.0 / std::fabs(kappa);
    }
  }
}

} // namespace planning
} // namespace apollo