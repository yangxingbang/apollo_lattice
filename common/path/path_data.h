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
 * @file path_data.h
 **/

#pragma once

#include <list>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "modules/planning/common/path/discretized_path.h"
#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class PathData {
 public:
  enum class PathPointType {
    IN_LANE,
    OUT_ON_FORWARD_LANE,
    OUT_ON_REVERSE_LANE,
    OFF_ROAD,
    UNKNOWN,
  };

  PathData() = default;

  bool SetDiscretizedPath(DiscretizedPath path);

  bool SetFrenetPath(FrenetFramePath frenet_path);

  bool SetFrenetPath(FrenetFramePath frenet_path, const double init_trailer_theta);

  void SetReferenceLine(const ReferenceLine *reference_line);

  bool SetPathPointDecisionGuide(
      std::vector<std::tuple<double, PathPointType, double>>
          path_point_decision_guide);

  const DiscretizedPath &discretized_path() const;

  const FrenetFramePath &frenet_frame_path() const;

  const std::vector<std::tuple<double, PathPointType, double>>
      &path_point_decision_guide() const;

  common::PathPoint GetPathPointWithPathS(const double s) const;

  std::list<std::pair<DiscretizedPath, FrenetFramePath>> &path_data_history();

  /*
   * brief: this function will find the path_point in discretized_path whose
   * projection to reference line has s value closest to ref_s.
   */
  bool GetPathPointWithRefS(const double ref_s,
                            common::PathPoint *const path_point) const;

  bool LeftTrimWithRefS(const common::FrenetFramePoint &frenet_point);
  bool LeftTrimWithRefS(
      const common::FrenetFramePoint &frenet_point, const double init_trailer_theta);

  bool UpdateFrenetFramePath(const ReferenceLine *reference_line);

  void Clear();

  bool Empty() const;

  std::string DebugString() const;

  void set_path_label(const std::string &label);

  const std::string &path_label() const;

  void set_blocking_obstacle_id(const std::string &obs_id) {
    blocking_obstacle_id_ = obs_id;
  }
  const std::string &blocking_obstacle_id() const {
    return blocking_obstacle_id_;
  }

 private:
  /*
   * convert frenet path to cartesian path by reference line
   */
  bool SLToXY(const FrenetFramePath &frenet_path,
              DiscretizedPath *const discretized_path);
  bool SLToXY(const FrenetFramePath &frenet_path,
              const double init_trailer_theta,
              DiscretizedPath *const discretized_path);
  bool XYToSL(const DiscretizedPath &discretized_path,
              FrenetFramePath *const frenet_path);
  const ReferenceLine *reference_line_ = nullptr;
  DiscretizedPath discretized_path_;
  FrenetFramePath frenet_path_;
  /**
   * @brief speed decision generated by path analyzer for guiding speed limit
   * generation in speed bounds decider
   * @param tuple consists of s axis position on reference line; PathPointType
   * Enum; distance to closest obstacle
   */
  std::vector<std::tuple<double, PathPointType, double>>
      path_point_decision_guide_;
  std::list<std::pair<DiscretizedPath, FrenetFramePath>> path_data_history_;

  std::string path_label_ = "";
  std::string blocking_obstacle_id_;
};

}  // namespace planning
}  // namespace apollo
