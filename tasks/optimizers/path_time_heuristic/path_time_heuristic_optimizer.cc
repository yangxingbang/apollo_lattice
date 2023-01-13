/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file path_time_heuristic_optimizer.cc
 **/

#include "modules/planning/tasks/optimizers/path_time_heuristic/path_time_heuristic_optimizer.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/gridded_path_time_graph.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

PathTimeHeuristicOptimizer::PathTimeHeuristicOptimizer(const TaskConfig& config)
    : SpeedOptimizer(config) {
  CHECK(config.has_speed_heuristic_config());
  speed_heuristic_config_ = config.speed_heuristic_config();
}

bool PathTimeHeuristicOptimizer::SearchPathTimeGraph(
    SpeedData* speed_data) const {
  GriddedPathTimeGraph st_graph(
      reference_line_info_->st_graph_data(), dp_st_speed_config_,
      reference_line_info_->path_decision()->obstacles().Items(), init_point_);

  if (!st_graph.Search(speed_data).ok()) {
    AERROR << "failed to search graph with dynamic programming.";
    return false;
  }
  return true;
}

Status PathTimeHeuristicOptimizer::Process(
    const PathData& path_data, const common::TrajectoryPoint& init_point,
    SpeedData* const speed_data) {
  init_point_ = init_point;

  dp_st_speed_config_ = reference_line_info_->IsChangeLanePath()
                            ? speed_heuristic_config_.lane_change_speed_config()
                            : speed_heuristic_config_.default_speed_config();

  if (path_data.discretized_path().empty()) {
    std::string msg("Empty path data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!SearchPathTimeGraph(speed_data)) {
    const std::string msg(Name() +
                          ":Failed to search graph with dynamic programming.");
    AERROR << msg;
    RecordDebugInfo(*speed_data, reference_line_info_->mutable_st_graph_data()
                                     ->mutable_st_graph_debug());
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // jch 打印path的图形
  if(FLAGS_print_log_parse_speed){
    const auto& discretized_path = path_data.discretized_path();
    for(uint i=0;i<discretized_path.size();i++){  // << std::fixed << std::setprecision(n)放在一起是设置小数点位的精度，只有<< std::setprecision(n)，则n是保留有效位，只有std::fixed则是根据后面的数字类型的精度显示有效位
      AINFO << std::fixed << std::setprecision(2) << "log-parse::path lane, coord(x,y): " << discretized_path[i].x() << " , " << discretized_path[i].y();
    }
    // 打印st_boundaries，corners点的坐标
    const auto& st_boundaries = reference_line_info_->st_graph_data().st_boundaries();
    for(uint i=0;i<st_boundaries.size();i++){
      const auto& points = st_boundaries[i]->points();
      AINFO << "log-parse::obs st_boundary(id,corners(x,y)): " << st_boundaries[i]->id() << " ; " << std::fixed << std::setprecision(2) 
                << points[0].x() << " , " << points[0].y()
        << " ; " << points[1].x() << " , " << points[1].y()
        << " ; " << points[2].x() << " , " << points[2].y()
        << " ; " << points[3].x() << " , " << points[3].y();
    }
    // 打印speed_limit_
    const auto& speed_limit = reference_line_info_->st_graph_data().speed_limit().speed_limit_points();
    for(uint i=0;i<speed_limit.size();i++){
      AINFO << "log-parse::speed_limit(s,lim_v): " << std::fixed << std::setprecision(2) << speed_limit[i].first << " , " << speed_limit[i].second;
    }
    // 打印cruise_speed

    // 打印st_drivable_boundary

    // 打印DP搜索结果的s
    for(uint i=0;i<(*speed_data).size();i++){
      AINFO << "log-parse::DP speed_data(t,s,v): " << std::fixed << std::setprecision(2) << (*speed_data)[i].t() << " , " << (*speed_data)[i].s() << " , " << (*speed_data)[i].v();
    }
    // 打印DP搜索结果的v
  }

  ////////////////////////////////

  RecordDebugInfo(
      *speed_data,
      reference_line_info_->mutable_st_graph_data()->mutable_st_graph_debug());
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
