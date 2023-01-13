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
 * @file
 **/

#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_limit_decider.h"

#include <algorithm>
#include <limits>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/decision.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::TrajectoryPoint;

SpeedLimitDecider::SpeedLimitDecider(const SpeedBoundsDeciderConfig& config,
                                     const ReferenceLine& reference_line,
                                     const PathData& path_data)
    : speed_bounds_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
}

Status SpeedLimitDecider::GetSpeedLimits(
    const IndexedList<std::string, Obstacle>& obstacles,
    const TrajectoryPoint& init_point,
    SpeedLimit* const speed_limit_data) const {
  CHECK_NOTNULL(speed_limit_data);

  const auto& discretized_path = path_data_.discretized_path();
  const auto& frenet_path = path_data_.frenet_frame_path();

  // Get crosswalk overlaps, wsy1122
  const std::vector<hdmap::PathOverlap>& crosswalk_overlaps = 
      reference_line_.map_path().crosswalk_overlaps();
  ADEBUG << "Size of crosswalk_overlaps is : " << crosswalk_overlaps.size();

  double prev_speed = init_point.v();
  double prev_accel = init_point.a();
  ADEBUG << "init_point.v = " << init_point.v() << 
      ", init_point.a = " << init_point.a();

  for (uint32_t i = 0; i < discretized_path.size(); ++i) {
    const double path_s = discretized_path.at(i).s();
    const double reference_line_s = frenet_path.at(i).s();
    if (reference_line_s > reference_line_.Length()) {
      AWARN << "path w.r.t. reference line at [" << reference_line_s
            << "] is LARGER than reference_line_ length ["
            << reference_line_.Length() << "]. Please debug before proceeding.";
      break;
    }

    // (1) speed limit from map
    double speed_limit_coeff = 0.93;
    double speed_limit_from_reference_line =
        reference_line_.GetSpeedLimitFromS(reference_line_s) * speed_limit_coeff;  
    double extend_map_speed_limit_distance = 30.0;
    for(double temp_reference_line_s=reference_line_s;temp_reference_line_s<reference_line_.Length();temp_reference_line_s+=2.0){
      double temp_speed_limit_from_reference_line =
        reference_line_.GetSpeedLimitFromS(temp_reference_line_s) * speed_limit_coeff;  
      speed_limit_from_reference_line = std::fmin(speed_limit_from_reference_line,
                                                   temp_speed_limit_from_reference_line);
      if((temp_reference_line_s-reference_line_s)>extend_map_speed_limit_distance){
        break;
      }
    }
    

    // (2) speed limit from path curvature
    //  -- 2.1: limit by centripetal force (acceleration)
    double speed_limit_from_centripetal_acc =
        std::sqrt(speed_bounds_config_.max_centric_acceleration_limit() /
                  std::fmax(std::fabs(discretized_path.at(i).kappa()),
                            speed_bounds_config_.minimal_kappa()));
    //jch: travel range [point-kappa_start_index, point+kappa_end_index],select the lowest speed limit as the point speed limit
    double kappa_start_index = 10;
    double kappa_end_index = 25;
    double speed_limit_from_centripetal_acc_back = 100;
    if((i+kappa_end_index)<discretized_path.size() && (i-kappa_start_index)>0){
      for(int j=i-kappa_start_index;j<(i+kappa_end_index);j++){
        speed_limit_from_centripetal_acc_back =
        std::sqrt(speed_bounds_config_.max_centric_acceleration_limit() /  
                  std::fmax(std::fabs(discretized_path.at(j).kappa()),
                            speed_bounds_config_.minimal_kappa()));
        speed_limit_from_centripetal_acc = std::min(speed_limit_from_centripetal_acc, speed_limit_from_centripetal_acc_back);
      }
    }
    if(speed_limit_from_centripetal_acc < 5.5){ 
      speed_limit_from_centripetal_acc = speed_limit_from_centripetal_acc * 0.85;
      if(speed_limit_from_centripetal_acc < 4.5){  // jch: Lower speed limits for over-curvature curves
        speed_limit_from_centripetal_acc = speed_limit_from_centripetal_acc * 0.9;
        if(speed_limit_from_centripetal_acc < 3.5){  // jch: Lower speed limits for over-curvature curves
          speed_limit_from_centripetal_acc = speed_limit_from_centripetal_acc * 0.9;
        }
      }
    }
    // AINFO << "s(): " << discretized_path.at(i).s() << "  speed_limit_from_centripetal_acc : " << speed_limit_from_centripetal_acc;

    // (3) speed limit from nudge obstacles
    // TODO(all): in future, expand the speed limit not only to obstacles with
    // nudge decisions.
    // double speed_limit_from_nearby_obstacles =
    //     std::numeric_limits<double>::max();
    double speed_limit_from_nearby_obstacles = 30;  //jch
    const double collision_safety_range =
        speed_bounds_config_.collision_safety_range();
    for (const auto* ptr_obstacle : obstacles.Items()) {
      if (ptr_obstacle->IsVirtual()) {
        continue;
      }
      if (!ptr_obstacle->LateralDecision().has_nudge()) {
        continue;
      }

      /* ref line:
       * -------------------------------
       *    start_s   end_s
       * ------|  adc   |---------------
       * ------------|  obstacle |------
       */

      // TODO(all): potential problem here;
      // frenet and cartesian coordinates are mixed.
      const double vehicle_front_s =
          reference_line_s + vehicle_param_.front_edge_to_center();
      const double vehicle_back_s =
          reference_line_s - vehicle_param_.back_edge_to_center();
      const double obstacle_front_s =
          ptr_obstacle->PerceptionSLBoundary().end_s();
      const double obstacle_back_s =
          ptr_obstacle->PerceptionSLBoundary().start_s();

      if (vehicle_front_s < obstacle_back_s ||
          vehicle_back_s > obstacle_front_s) {
        continue;
      }

      const auto& nudge_decision = ptr_obstacle->LateralDecision().nudge();

      // Please notice the differences between adc_l and frenet_point_l
      const double frenet_point_l = frenet_path.at(i).l();

      // obstacle is on the right of ego vehicle (at path point i)
      bool is_close_on_left =
          (nudge_decision.type() == ObjectNudge::LEFT_NUDGE) &&
          (frenet_point_l - vehicle_param_.right_edge_to_center() -
               collision_safety_range <
           ptr_obstacle->PerceptionSLBoundary().end_l());

      // obstacle is on the left of ego vehicle (at path point i)
      bool is_close_on_right =
          (nudge_decision.type() == ObjectNudge::RIGHT_NUDGE) &&
          (ptr_obstacle->PerceptionSLBoundary().start_l() -
               collision_safety_range <
           frenet_point_l + vehicle_param_.left_edge_to_center());

      // TODO(all): dynamic obstacles do not have nudge decision
      if (is_close_on_left || is_close_on_right) {
        double nudge_speed_ratio = 1.0;
        if (ptr_obstacle->IsStatic()) {
          nudge_speed_ratio =
              speed_bounds_config_.static_obs_nudge_speed_ratio();
        } else {
          nudge_speed_ratio =
              speed_bounds_config_.dynamic_obs_nudge_speed_ratio();
        }
        speed_limit_from_nearby_obstacles =
            nudge_speed_ratio * speed_limit_from_reference_line;
        break;
      }
    }    

    // (4) speed limit from crosswalks, wsy1122
    const double pre_deceleration_distance_crosswalk_lv2 = 15.0;  //jch,for tongzhou test requirement
    const double pre_deceleration_distance_crosswalk_lv1 = 40.0;  //jch,for tongzhou test requirement
    const double delay_deceleration_distance_crosswalk_lv1 = 10.0;  //jch,for tongzhou test requirement
    const double delay_deceleration_distance_crosswalk_lv2 = 5.0;  //jch,for tongzhou test requirement
    // double speed_limit_from_crosswalks = std::numeric_limits<double>::max();
    double speed_limit_from_crosswalks = 30;   //jch
    const double vehicle_front_s =
        reference_line_s + vehicle_param_.front_edge_to_center();
    const double vehicle_back_s =
        reference_line_s - vehicle_param_.back_edge_to_center();
    double tmp_speed_limit_from_crosswalks = speed_limit_from_crosswalks;
    for (const hdmap::PathOverlap& crosswalk_overlap : crosswalk_overlaps) {
      if(crosswalk_overlap.object_id=="75267033" || crosswalk_overlap.object_id=="75775001"){  //jch,for shield tianjin high-speed loop crosswalk speed limit
        continue;
      }
        if (vehicle_front_s > 
            crosswalk_overlap.start_s - pre_deceleration_distance_crosswalk_lv1 && 
            vehicle_back_s < crosswalk_overlap.end_s + delay_deceleration_distance_crosswalk_lv1) {
              tmp_speed_limit_from_crosswalks = 8.33;
          if (vehicle_front_s > 
              crosswalk_overlap.start_s - pre_deceleration_distance_crosswalk_lv2 && 
              vehicle_back_s < crosswalk_overlap.end_s + delay_deceleration_distance_crosswalk_lv2) {
              tmp_speed_limit_from_crosswalks = FLAGS_crosswalks_speed_limit;
          }
        }
      speed_limit_from_crosswalks = std::min({tmp_speed_limit_from_crosswalks,speed_limit_from_crosswalks});
    }

    double curr_speed_limit = 0.0;
    if (FLAGS_enable_nudge_slowdown) {
      curr_speed_limit =
          std::fmax(speed_bounds_config_.lowest_speed(),
                    std::min({speed_limit_from_reference_line,
                              speed_limit_from_centripetal_acc,
                              speed_limit_from_nearby_obstacles,
                              speed_limit_from_crosswalks}));
    } else {
      curr_speed_limit =
          std::fmax(speed_bounds_config_.lowest_speed(),
                    std::min({speed_limit_from_reference_line,
                              speed_limit_from_centripetal_acc,
                              speed_limit_from_crosswalks}));
    }
    if(FLAGS_print_log_parse_speed){
      AINFO << "log-parse::speed_limit_decider(s,total,reference_line,centripetal_acc,nearby_obstacles,crosswalks): "  << std::fixed
            << path_s 
            << " , " << curr_speed_limit 
            << " , " << speed_limit_from_reference_line 
            << " , " << speed_limit_from_centripetal_acc 
            << " , " << speed_limit_from_nearby_obstacles 
            << " , " << speed_limit_from_crosswalks; 
    }
    ADEBUG << "At s = " << path_s << ", curr_speed_limit = " << curr_speed_limit;
    ADEBUG << "from_reference_line: " << speed_limit_from_reference_line
        << ", from_centripetal_acc: " << speed_limit_from_centripetal_acc
        << ", from_nearby_obstacles: " << speed_limit_from_nearby_obstacles
        << ", from_crosswalks: " << speed_limit_from_crosswalks;

    // adjust speed limit from initial speed and acceleration (wsy1104)
    // min_jerk need to be slightly larger than FLAGS_longitudinal_jerk_lower_bound.
    const double min_jerk = -3;
    ADEBUG << "min_jerk = " << min_jerk;
    const double ADC_speed_buffer = 0.3;
    double ADC_accel_buffer = prev_accel > 0 ?
        prev_accel * prev_accel / (-min_jerk) / 2 : 0;
    if (prev_speed + ADC_accel_buffer + ADC_speed_buffer > curr_speed_limit) {      
      if (i == 0) {
        curr_speed_limit = std::fmax(curr_speed_limit, 
                                     prev_speed + ADC_speed_buffer);     
      } else {
        double estimate_time = 
            (path_s - discretized_path.at(i - 1).s()) / prev_speed;
        double curr_accel = prev_accel + estimate_time * min_jerk;
        double curr_speed = prev_speed + prev_accel * estimate_time +
            min_jerk * estimate_time * estimate_time / 2;        
        curr_speed_limit = std::fmax(curr_speed_limit, 
                                     curr_speed + ADC_speed_buffer);
        prev_speed = curr_speed;
        prev_accel = curr_accel;
      }
    }
    ADEBUG << "Adjusted curr_speed_limit = " << curr_speed_limit;

    speed_limit_data->AppendSpeedLimit(path_s, curr_speed_limit);
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
