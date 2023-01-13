/******************************************************************************
 * Copyright 2021 The DeepWay Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file mini_overhang_path_optimizer.h
 **/

#pragma once

#include <vector>

#include "osqp/include/osqp.h"

namespace apollo {
namespace planning {

class QPSolver {
 public:
  QPSolver() = delete;
  explicit QPSolver(const int& max_iter);
  bool OK() const { return status_; }
  virtual bool Optimize();

 private:
  std::vector<double> Solution() { return solution_; }
  virtual void CalculateKernel(std::vector<c_float>* P_data, 
                               std::vector<c_int>* P_indices, 
                               std::vector<c_int>* P_indptr) = 0;
  virtual void CalculateOffset(std::vector<c_float>* q) = 0;
  virtual void CalculateAffineConstraint(
      std::vector<c_float>* A_data, std::vector<c_int>* A_indices, 
      std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds, 
      std::vector<c_float>* upper_bounds) = 0;
  virtual OSQPSettings* SolverDefaultSetting();
  virtual void SetWarmStartX() = 0;
  virtual void SaveLastSolution() = 0;
  virtual void ClearLastSolution() = 0;
 private:
  OSQPData* FormulateProblem();
  void FreeData(OSQPData* data);
  void WarmStart(OSQPWorkspace* work);
  template <typename T>
  T* CopyData(const std::vector<T>& vec) {
    T* data = new T[vec.size()];
    memcpy(data, vec.data(), sizeof(T) * vec.size());
    return data;
  }

 protected:
  int max_iter_;
  bool status_;
  size_t kernel_dim_;
  std::vector<double> solution_;
  std::vector<c_float> start_x_;
};

} // namespace planning
} // namespace apollo