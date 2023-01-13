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
#include "modules/planning/planning_component.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/navi_planning.h"
#include "modules/planning/on_lane_planning.h"
#include "modules/planning/common/configs/truck_config_helper.h"

namespace apollo {
namespace planning {

using apollo::hdmap::HDMapUtil;
using apollo::perception::TrafficLightDetection;
using apollo::relative_map::MapMsg;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::canbus::Chassis;   //jch 20221025
using apollo::common::TrajectoryPoint;  //jch 20221025
using apollo::common::VehicleStateProvider;  //jch 20221025

bool PlanningComponent::Init() {
  TruckConfigHelper::Init();
  if (FLAGS_use_navigation_mode) {
    planning_base_ = std::make_unique<NaviPlanning>();
  } else {
    planning_base_ = std::make_unique<OnLanePlanning>();
  }

  CHECK(apollo::cyber::common::GetProtoFromFile(FLAGS_planning_config_file,
                                                &config_))
      << "failed to load planning config file " << FLAGS_planning_config_file;
  planning_base_->Init(config_);

  routing_reader_ = node_->CreateReader<RoutingResponse>(
      FLAGS_routing_response_topic,
      [this](const std::shared_ptr<RoutingResponse>& routing) {
        AINFO << "Received routing data: run routing callback."
              << routing->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        routing_.CopyFrom(*routing);
      });
  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      FLAGS_traffic_light_detection_topic,
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        ADEBUG << "Received traffic light data: run traffic light callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        traffic_light_.CopyFrom(*traffic_light);
      });

  pad_msg_reader_ = node_->CreateReader<PadMessage>(
      FLAGS_planning_pad_topic,
      [this](const std::shared_ptr<PadMessage>& pad_msg) {
        ADEBUG << "Received pad data: run pad callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        pad_msg_.CopyFrom(*pad_msg);
      });

  if (FLAGS_use_navigation_mode) {
    relative_map_reader_ = node_->CreateReader<MapMsg>(
        FLAGS_relative_map_topic,
        [this](const std::shared_ptr<MapMsg>& map_message) {
          ADEBUG << "Received relative map data: run relative map callback.";
          std::lock_guard<std::mutex> lock(mutex_);
          relative_map_.CopyFrom(*map_message);
        });
  }
  planning_writer_ =
      node_->CreateWriter<ADCTrajectory>(FLAGS_planning_trajectory_topic);

  rerouting_writer_ =
      node_->CreateWriter<RoutingRequest>(FLAGS_routing_request_topic);

  return true;
}

bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  CHECK(prediction_obstacles != nullptr);

  // check and process possible rerouting request
  CheckRerouting();

  // process fused input data
  local_view_.prediction_obstacles = prediction_obstacles;
  local_view_.chassis = chassis;
  local_view_.localization_estimate = localization_estimate;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.routing ||
        hdmap::PncMap::IsNewRouting(*local_view_.routing, routing_)) {
      local_view_.routing =
          std::make_shared<routing::RoutingResponse>(routing_);
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.traffic_light =
        std::make_shared<TrafficLightDetection>(traffic_light_);
    local_view_.relative_map = std::make_shared<MapMsg>(relative_map_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.pad_msg = std::make_shared<PadMessage>(pad_msg_);
    ADEBUG << "bcm_pullovercmd: " << chassis->bcm().bcm_pullovercmd();
    if (chassis->bcm().bcm_pullovercmd() == 1) {
      local_view_.pad_msg->set_action(DrivingAction::PULL_OVER);
    } else {
      local_view_.pad_msg->set_action(DrivingAction::NONE);
    }
    ADEBUG << "pad_msg.action: " << local_view_.pad_msg->action();
  }

  // if using lattice planner， the concept of scenario and task is gone. 
  
  if (false) {
    auto fake_predict_obstacle_ptr = 
            local_view_.prediction_obstacles->add_prediction_obstacle();
    auto fake_obstacle_ptr = 
            fake_predict_obstacle_ptr->mutable_perception_obstacle();
    auto position_ptr = fake_obstacle_ptr->mutable_position();
    auto velocity_ptr = fake_obstacle_ptr->mutable_velocity();
    position_ptr->set_x(523628.58);
    position_ptr->set_y(4303418.80);
    position_ptr->set_z(0.0);
    velocity_ptr->set_x(0.5);
    velocity_ptr->set_y(0.0);
    velocity_ptr->set_z(0.0);
    fake_obstacle_ptr->set_id(0);
    fake_obstacle_ptr->set_theta(-0.845);
    fake_obstacle_ptr->set_length(4.0);
    fake_obstacle_ptr->set_width(2.0);
    fake_obstacle_ptr->set_height(1.8);
    fake_obstacle_ptr->set_type(apollo::perception::PerceptionObstacle::VEHICLE);
    fake_predict_obstacle_ptr->set_is_static(true);
  }

  // if (FLAGS_enable_use_fake_obstacle_for_demo) {
  //   auto fake_predict_obstacle_ptr = 
  //           local_view_.prediction_obstacles->add_prediction_obstacle();
  //   auto fake_obstacle_ptr = 
  //           fake_predict_obstacle_ptr->mutable_perception_obstacle();
  //   auto position_ptr = fake_obstacle_ptr->mutable_position();
  //   auto velocity_ptr = fake_obstacle_ptr->mutable_velocity();
  //   position_ptr->set_x(523933.29);
  //   position_ptr->set_y(4303048.82);
  //   position_ptr->set_z(0.0);
  //   velocity_ptr->set_x(0.0);
  //   velocity_ptr->set_y(0.0);
  //   velocity_ptr->set_z(0.0);
  //   fake_obstacle_ptr->set_id(1);
  //   fake_obstacle_ptr->set_theta(0);
  //   fake_obstacle_ptr->set_length(4.0);
  //   fake_obstacle_ptr->set_width(0.5);
  //   fake_obstacle_ptr->set_height(0.5);
  //   fake_obstacle_ptr->set_type(apollo::perception::PerceptionObstacle::UNKNOWN_UNMOVABLE);
  //   fake_predict_obstacle_ptr->set_is_static(true);
  // }
  

  if (!CheckInput()) {
    AERROR << "Input check failed";
    return false;
  }

  ADCTrajectory adc_trajectory_pb;
  AINFO << "------------------------------------------------------------------------------------------";
  AINFO << "log-parse::PlanningComponent::Proc_start";
  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);
  if(FLAGS_print_log_parse_speed){
    for(int i=0;i<adc_trajectory_pb.trajectory_point_size();i++){
      auto trajectory_point = adc_trajectory_pb.trajectory_point(i);
      AINFO << "log-parse::trajectory(t,x,y): "  << std::fixed << std::setprecision(2) << trajectory_point.relative_time() << " , " 
      << trajectory_point.path_point().x() << " , " << trajectory_point.path_point().y();  
    }
  }
  AINFO << "log-parse::PlanningComponent::Proc_end";

  // 20221024,jiachunhui add for turn light
  trajectory_analyzer_.reset(new TrajectoryAnalyzer(&adc_trajectory_pb));
  if(chassis->driving_mode() != Chassis::COMPLETE_AUTO_DRIVE){
    turn_right_switch_ = false;
    turn_left_switch_ = false;
    turn_right_ = false;
    turn_left_ = false;
  }
  // turn right
  TrajectoryPoint turn_right_point =
      trajectory_analyzer_->QueryNearestPointByPosition(
          turn_right_pre_point_x_,turn_right_pre_point_y_);
  double distance_p1_1 = std::sqrt(std::pow(turn_right_point.path_point().x()-turn_right_pre_point_x_,2)
          + std::pow(turn_right_point.path_point().y()-turn_right_pre_point_y_,2));
  // AINFO << "distance_p1_1: " <<distance_p1_1;
  if(distance_p1_1 < 1.0){ // the turn_right_pre_point is near by trajectory
    turn_right_switch_ = true;
  }
  if(turn_right_switch_ == true){
    double distance_p1_2 = std::sqrt(std::pow(VehicleStateProvider::Instance()->x()-turn_right_pre_point_x_,2)
          + std::pow(VehicleStateProvider::Instance()->y()-turn_right_pre_point_y_,2));
    double distance_triger_turn = VehicleStateProvider::Instance()->linear_velocity() * 5.0;  //提前2s触发
    // AINFO << "distance_p1_2: " <<distance_p1_2;
    // AINFO << "distance_triger_turn: " <<distance_triger_turn;
    if(distance_p1_2 < distance_triger_turn){
      turn_right_ = true;
    }
    double distance_p1_3 = std::sqrt(std::pow(VehicleStateProvider::Instance()->x()-turn_right_end_point_x_,2)
          + std::pow(VehicleStateProvider::Instance()->y()-turn_right_end_point_y_,2));
    // AINFO << "distance_p1_3: " <<distance_p1_3;
    if(distance_p1_3 < 7){  //距离turn_right_end_point很近的时候就不要再打转向灯了
      turn_right_switch_ = false;
      turn_right_ = false;
    }  
  }
  // turn left
  TrajectoryPoint turn_left_point =
      trajectory_analyzer_->QueryNearestPointByPosition(
          turn_left_pre_point_x_,turn_left_pre_point_y_);
  double distance_p2_1 = std::sqrt(std::pow(turn_left_point.path_point().x()-turn_left_pre_point_x_,2)
          + std::pow(turn_left_point.path_point().y()-turn_left_pre_point_y_,2));
  // AINFO << "distance_p2_1: " <<distance_p2_1;
  if(distance_p2_1 < 1.0){ // the turn_left_pre_point is near by trajectory
    turn_left_switch_ = true;
  }
  if(turn_left_switch_ == true){
    double distance_p2_2 = std::sqrt(std::pow(VehicleStateProvider::Instance()->x()-turn_left_pre_point_x_,2)
          + std::pow(VehicleStateProvider::Instance()->y()-turn_left_pre_point_y_,2));
    // AINFO << "distance_p2_2: " <<distance_p2_2;
    double distance_triger_turn = VehicleStateProvider::Instance()->linear_velocity() * 5.0;  //提前2s触发
    // AINFO << "distance_triger_turn: " <<distance_triger_turn;
    if(distance_p2_2 < distance_triger_turn){
      turn_left_ = true;
    }
    double distance_p2_3 = std::sqrt(std::pow(VehicleStateProvider::Instance()->x()-turn_left_end_point_x_,2)
          + std::pow(VehicleStateProvider::Instance()->y()-turn_left_end_point_y_,2));
    // AINFO << "distance_p2_3: " <<distance_p2_3;
    if(distance_p2_3 < 7){  //距离turn_left_end_point很近的时候就不要再打转向灯了
      turn_left_switch_ = false;
      turn_left_ = false;
    }  
  }
  // AINFO << "turn right light: " << turn_right_;
  // AINFO << "turn left light: " << turn_left_;
  auto vehicle_signal = adc_trajectory_pb.mutable_decision()->mutable_vehicle_signal();
  if(turn_left_){
    vehicle_signal->set_turn_signal(common::VehicleSignal::TURN_LEFT);
  }else{
    if(turn_right_){
      vehicle_signal->set_turn_signal(common::VehicleSignal::TURN_RIGHT);
    }
  }
  // AINFO << "turn signal： " << vehicle_signal->turn_signal();
  // end add for turn light

  auto start_time = adc_trajectory_pb.header().timestamp_sec();
  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);

  // modify trajectory relative time due to the timestamp change in header
  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
  }
  planning_writer_->Write(adc_trajectory_pb);

  // record in history
  auto* history = History::Instance();
  history->Add(adc_trajectory_pb);

  return true;
}

void PlanningComponent::CheckRerouting() {
  auto* rerouting = PlanningContext::Instance()
                        ->mutable_planning_status()
                        ->mutable_rerouting();
  if (!rerouting->need_rerouting()) {
    return;
  }
  common::util::FillHeader(node_->Name(), rerouting->mutable_routing_request());
  rerouting->set_need_rerouting(false);
  rerouting_writer_->Write(rerouting->routing_request());
}

bool PlanningComponent::CheckInput() {
  ADCTrajectory trajectory_pb;
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  if (local_view_.localization_estimate == nullptr) {
    not_ready->set_reason("localization not ready");
  } else if (local_view_.chassis == nullptr) {
    not_ready->set_reason("chassis not ready");
  } else if (HDMapUtil::BaseMapPtr() == nullptr) {
    not_ready->set_reason("map not ready");
  } else {
    // nothing
  }

  if (FLAGS_use_navigation_mode) {
    if (!local_view_.relative_map->has_header()) {
      not_ready->set_reason("relative map not ready");
    }
  } else {
    if (!local_view_.routing->has_header()) {
      not_ready->set_reason("routing not ready");
    }
  }

  if (not_ready->has_reason()) {
    AERROR << not_ready->reason() << "; skip the planning cycle.";
    common::util::FillHeader(node_->Name(), &trajectory_pb);
    planning_writer_->Write(trajectory_pb);
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
