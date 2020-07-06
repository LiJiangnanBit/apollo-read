/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_decider.h"

#include <algorithm>
#include <string>

#include "modules/planning/common/obstacle_blocking_analyzer.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

constexpr double kIntersectionClearanceDist = 20.0;
constexpr double kJunctionClearanceDist = 15.0;

PathLaneBorrowDecider::PathLaneBorrowDecider(const TaskConfig& config)
    : Decider(config) {}

Status PathLaneBorrowDecider::Process(
  // 执行完这个函数之后，实际上就是修改了 PlanningContext::Instance()
                                          // ->mutable_planning_status()
                                          // ->mutable_path_decider();的状态（是否要借道，哪个方向可以）
                                          // 另外还有这条车道的is_path_lane_borrow
                                          // 是否借道没有考虑动态障碍物，单纯是通过左右是否是实线判断的。
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // skip path_lane_borrow_decider if reused path
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    // for debug
    AINFO << "skip due to reusing path";
    return Status::OK();
  }

  // By default, don't borrow any lane.
  reference_line_info->set_is_path_lane_borrow(false);
  // Check if lane-borrowing is needed, if so, borrow lane.
  if (Decider::config_.path_lane_borrow_decider_config()
          .allow_lane_borrowing() &&
      IsNecessaryToBorrowLane(*frame, *reference_line_info)) {
    reference_line_info->set_is_path_lane_borrow(true);
  }
  return Status::OK();
}

bool PathLaneBorrowDecider::IsNecessaryToBorrowLane(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  auto* mutable_path_decider_status = PlanningContext::Instance()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  if (mutable_path_decider_status->is_in_path_lane_borrow_scenario()) {
    // If originally borrowing neighbor lane:
    if (mutable_path_decider_status->able_to_use_self_lane_counter() >= 6) {
      // If have been able to use self-lane for some time, then switch to
      // non-lane-borrowing.
      mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
      mutable_path_decider_status->clear_decided_side_pass_direction();
      AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";
    }
  } else {
    // If originally not borrowing neighbor lane:
    ADEBUG << "Blocking obstacle ID["
           << mutable_path_decider_status->front_static_obstacle_id() << "]";
    // ADC requirements check for lane-borrowing:
    if (!HasSingleReferenceLine(frame)) {
      // 注释里写着： This function is to prevent lane-borrowing during lane-changing.
      // 意味着，只有在换道的时候，frame里面才有多于一条参考线？
      // 只有在“仅有一条参考线”的时候才会借道？如果有多条参考线，则换道？
      return false;
    }
    if (!IsWithinSidePassingSpeedADC(frame)) {
      // 如果要lane-borrow，速度要低于某个数值。
      return false;
    }

    // Obstacle condition check for lane-borrowing:
    if (!IsBlockingObstacleFarFromIntersection(reference_line_info)) {
      // IsBlockingObstacleFarFromIntersection：如果前方没有障碍物，或者障碍物距离路口比较远，返回true，此时可以sidepass。
      return false;
    }
    if (!IsLongTermBlockingObstacle()) {
      return false;
    }
    if (!IsBlockingObstacleWithinDestination(reference_line_info)) {
      // 如果目标点（？）在障碍物和车之间，不需要借道，因为到达障碍物之前就停了。
      return false;
    }
    if (!IsSidePassableObstacle(reference_line_info)) {
      // 如果障碍物一会儿就开走了，不需要借道。
      return false;
    }

    // switch to lane-borrowing
    // set side-pass direction
    const auto& path_decider_status =
        PlanningContext::Instance()->planning_status().path_decider();
    if (path_decider_status.decided_side_pass_direction().empty()) {
      // first time init decided_side_pass_direction
      bool left_borrowable;
      bool right_borrowable;
      // 从车头位置开始检查，到设定的一个距离。对每个点，检查左边右边是否是实线。如果是，则不可向这个方向借道。输出两个bool，表示能否
      // 左右借道。
      CheckLaneBorrow(reference_line_info, &left_borrowable, &right_borrowable);
      if (!left_borrowable && !right_borrowable) {
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
        return false;
      } else {
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
        if (left_borrowable) {
          mutable_path_decider_status->add_decided_side_pass_direction(
              PathDeciderStatus::LEFT_BORROW);
        }
        if (right_borrowable) {
          mutable_path_decider_status->add_decided_side_pass_direction(
              PathDeciderStatus::RIGHT_BORROW);
        }
      }
    }

    AINFO << "Switch from SELF-LANE path to LANE-BORROW path.";
  }
  return mutable_path_decider_status->is_in_path_lane_borrow_scenario();
}

// This function is to prevent lane-borrowing during lane-changing.
// TODO(jiacheng): depending on our needs, may allow lane-borrowing during
//                 lane-change.
bool PathLaneBorrowDecider::HasSingleReferenceLine(const Frame& frame) {
  return frame.reference_line_info().size() == 1;
}

bool PathLaneBorrowDecider::IsWithinSidePassingSpeedADC(const Frame& frame) {
  return frame.PlanningStartPoint().v() < FLAGS_lane_borrow_max_speed;
}

bool PathLaneBorrowDecider::IsLongTermBlockingObstacle() {
  if (PlanningContext::Instance()
          ->planning_status()
          .path_decider()
          .front_static_obstacle_cycle_counter() >=
      FLAGS_long_term_blocking_obstacle_cycle_threshold) {
    ADEBUG << "The blocking obstacle is long-term existing.";
    return true;
  } else {
    ADEBUG << "The blocking obstacle is not long-term existing.";
    return false;
  }
}

bool PathLaneBorrowDecider::IsBlockingObstacleWithinDestination(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      PlanningContext::Instance()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().start_s();
  double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
  ADEBUG << "ADC is at s = " << adc_end_s;
  ADEBUG << "Destination is at s = "
         << reference_line_info.SDistanceToDestination() + adc_end_s;
  if (blocking_obstacle_s - adc_end_s >
      reference_line_info.SDistanceToDestination()) {
    return false;
  }
  return true;
}

bool PathLaneBorrowDecider::IsBlockingObstacleFarFromIntersection(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      PlanningContext::Instance()->planning_status().path_decider();
      // path_decider的类型定义：
    //       message PathDeciderStatus {
    //   enum LaneBorrowDirection {
    //     LEFT_BORROW = 1;   // borrow left neighbor lane
    //     RIGHT_BORROW = 2;  // borrow right neighbor lane
    //   }
    //   optional int32 front_static_obstacle_cycle_counter = 1 [default = 0];
    //   optional int32 able_to_use_self_lane_counter = 2 [default = 0];
    //   optional bool is_in_path_lane_borrow_scenario = 3 [default = false];
    //   optional string front_static_obstacle_id = 4 [default = ""];
    //   repeated LaneBorrowDirection decided_side_pass_direction = 5;
    // }
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  // Get blocking obstacle's s.
  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().end_s();
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
  // Get intersection's s and compare with threshold.
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  for (const auto& overlap : first_encountered_overlaps) {
    ADEBUG << overlap.first << ", " << overlap.second.DebugString();
    // if (// overlap.first != ReferenceLineInfo::CLEAR_AREA &&
    // overlap.first != ReferenceLineInfo::CROSSWALK &&
    // overlap.first != ReferenceLineInfo::PNC_JUNCTION &&
    if (overlap.first != ReferenceLineInfo::SIGNAL &&
        overlap.first != ReferenceLineInfo::STOP_SIGN) {
      continue;
    }

    auto distance = overlap.second.start_s - blocking_obstacle_s;
    if (overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::STOP_SIGN) {
      if (distance < kIntersectionClearanceDist) {
        ADEBUG << "Too close to signal intersection (" << distance
               << "m); don't SIDE_PASS.";
        return false;
      }
    } else {
      if (distance < kJunctionClearanceDist) {
        ADEBUG << "Too close to overlap_type[" << overlap.first << "] ("
               << distance << "m); don't SIDE_PASS";
        return false;
      }
    }
  }

  return true;
}

bool PathLaneBorrowDecider::IsSidePassableObstacle(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      PlanningContext::Instance()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return false;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return false;
  }

  return IsNonmovableObstacle(reference_line_info, *blocking_obstacle);
}

void PathLaneBorrowDecider::CheckLaneBorrow(
    const ReferenceLineInfo& reference_line_info,
    bool* left_neighbor_lane_borrowable, bool* right_neighbor_lane_borrowable) {
      // 从车头位置开始检查，到设定的一个距离。对每个点，检查左边右边是否是实线。如果是，则不可向这个方向借道。输出两个bool，表示能否
      // 左右借道。
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  *left_neighbor_lane_borrowable = true;
  *right_neighbor_lane_borrowable = true;

  static constexpr double kLookforwardDistance = 100.0;
  double check_s = reference_line_info.AdcSlBoundary().end_s();
  const double lookforward_distance =
      std::min(check_s + kLookforwardDistance, reference_line.Length());
  while (check_s < lookforward_distance) {
    auto ref_point = reference_line.GetNearestReferencePoint(check_s);
    if (ref_point.lane_waypoints().empty()) {
      // 
      *left_neighbor_lane_borrowable = false;
      *right_neighbor_lane_borrowable = false;
      return;
    }

    const auto waypoint = ref_point.lane_waypoints().front();
    hdmap::LaneBoundaryType::Type lane_boundary_type =
        hdmap::LaneBoundaryType::UNKNOWN;

    if (*left_neighbor_lane_borrowable) {
      lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
      if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
          lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
        *left_neighbor_lane_borrowable = false;
      }
      ADEBUG << "s[" << check_s << "] left_lane_boundary_type["
             << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
    }
    if (*right_neighbor_lane_borrowable) {
      lane_boundary_type = hdmap::RightBoundaryType(waypoint);
      if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
          lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
        *right_neighbor_lane_borrowable = false;
      }
      ADEBUG << "s[" << check_s << "] right_neighbor_lane_borrowable["
             << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
    }
    check_s += 2.0;
  }
}

}  // namespace planning
}  // namespace apollo
