/******************************************************************************
 * Copyright 2021 The DeepWay Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/planning/math/qpsolver/qp_solver.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

QPSolver::QPSolver(const int& max_iter) : max_iter_ (max_iter), status_(true) {}

bool QPSolver::Optimize() {
  OSQPData* data = FormulateProblem();
  if (status_ == false) return false;

  OSQPSettings* settings = SolverDefaultSetting();
  settings->max_iter = max_iter_;

  OSQPWorkspace* osqp_work = nullptr;
  osqp_work = osqp_setup(data, settings);

  SetWarmStartX();
  WarmStart(osqp_work);

  osqp_solve(osqp_work);

  auto status = osqp_work->info->status_val;

  if (status < 0 || (status != 1 && status != 2)) {
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    ClearLastSolution();
    status_ = false;
    AERROR << "qp solver status = " << status;
    return false;
  } else if (osqp_work->solution == nullptr) {
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    ClearLastSolution();
    status_ = false;
    AERROR << "osqp_work->solution == nullptr";
    return false;
  }

  solution_.resize(kernel_dim_);
  for (size_t i = 0; i < kernel_dim_; ++i) {
    solution_[i] = osqp_work->solution->x[i];
  }
  SaveLastSolution();
  // clean up
  osqp_cleanup(osqp_work);
  FreeData(data);
  c_free(settings);
  status_ = true;
  return true;
}

OSQPSettings* QPSolver::SolverDefaultSetting() {
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->polish = true;
  settings->verbose = false;
  settings->scaled_termination = false;
  settings->warm_start = true;
  return settings;
}

OSQPData* QPSolver::FormulateProblem() {
  // calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr,
                            &lower_bounds, &upper_bounds);

  // calculate offset
  std::vector<c_float> q;
  CalculateOffset(&q);

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  if (lower_bounds.size() != upper_bounds.size()) {
    status_ = false;
    AERROR << "lower_bounds.size() != upper_bounds.size()";
    return data;
  }
  size_t num_affine_constraint = lower_bounds.size();

  data->n = kernel_dim_;
  data->m = num_affine_constraint;
  data->P = csc_matrix(kernel_dim_, kernel_dim_, P_data.size(), CopyData(P_data),
                       CopyData(P_indices), CopyData(P_indptr));
  data->q = CopyData(q);
  data->A = 
      csc_matrix(num_affine_constraint, kernel_dim_, A_data.size(),
                 CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
  data->l = CopyData(lower_bounds);
  data->u = CopyData(upper_bounds);
  return data;
}

void QPSolver::FreeData(OSQPData* data) {
  delete[] data->q;
  delete[] data->l;
  delete[] data->u;

  delete[] data->P->i;
  delete[] data->P->p;
  delete[] data->P->x;

  delete[] data->A->i;
  delete[] data->A->p;
  delete[] data->A->x;
}

void QPSolver::WarmStart(OSQPWorkspace* work) {
  if (start_x_.size() != kernel_dim_) return;
  c_float* star_x = CopyData(start_x_);
  osqp_warm_start_x(work, star_x);
}

} // namespace planning
} // namespace apollo