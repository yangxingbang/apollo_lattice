/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/message/raw_message.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/proto/pad_msg.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/control/common/trajectory_analyzer.h"  //jch 20221025
#include "modules/common/vehicle_state/vehicle_state_provider.h"  //jch 20221025

namespace apollo {
namespace planning {

using apollo::control::TrajectoryAnalyzer;  //jch 20221025

class PlanningComponent final
    : public cyber::Component<prediction::PredictionObstacles, canbus::Chassis,
                              localization::LocalizationEstimate> {
 public:
  PlanningComponent() = default;

  ~PlanningComponent() = default;

 public:
  bool Init() override;

  bool Proc(const std::shared_ptr<prediction::PredictionObstacles>&
                prediction_obstacles,
            const std::shared_ptr<canbus::Chassis>& chassis,
            const std::shared_ptr<localization::LocalizationEstimate>&
                localization_estimate) override;

 private:
  void CheckRerouting();
  bool CheckInput();

 private:
  std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>>
      traffic_light_reader_;
  std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;
  std::shared_ptr<cyber::Reader<planning::PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<relative_map::MapMsg>> relative_map_reader_;

  std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;
  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> rerouting_writer_;

  std::mutex mutex_;
  perception::TrafficLightDetection traffic_light_;
  routing::RoutingResponse routing_;
  planning::PadMessage pad_msg_;
  relative_map::MapMsg relative_map_;

  LocalView local_view_;

  std::unique_ptr<PlanningBase> planning_base_;

  PlanningConfig config_;
  
  //20221024 jiachunhui
  bool turn_right_switch_ = false;
  bool turn_right_ = false;
  double turn_right_pre_point_x_ = 471025.19;
  double turn_right_pre_point_y_ = 4399127.12;
  double turn_right_end_point_x_ = 471026.83;
  double turn_right_end_point_y_ = 4399069.76;
 

  bool turn_left_ = false;
  bool turn_left_switch_ = false;
  double turn_left_pre_point_x_ = 471059.27;
  double turn_left_pre_point_y_ = 4398674.84;
  double turn_left_end_point_x_ = 471066.56;
  double turn_left_end_point_y_ = 4398632.00;
  std::unique_ptr<TrajectoryAnalyzer> trajectory_analyzer_;
};

CYBER_REGISTER_COMPONENT(PlanningComponent)

}  // namespace planning
}  // namespace apollo
