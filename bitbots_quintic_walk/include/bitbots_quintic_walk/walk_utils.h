#ifndef BITBOTS_MOTION_BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_UTILS_H_
#define BITBOTS_MOTION_BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_UTILS_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace bitbots_quintic_walk {

enum WalkState {
  PAUSED,
  WALKING,
  IDLE,
  START_MOVEMENT,
  STOP_MOVEMENT,
  START_STEP,
  STOP_STEP,
  KICK
};

struct WalkRequest {
  tf2::Transform orders;
  bool walkable_state;
};

struct WalkResponse {
  tf2::Transform support_foot_to_flying_foot;
  tf2::Transform support_foot_to_trunk;
  bool is_double_support;
  bool is_left_support_foot;

  double phase;
  double traj_time;
  double foot_distance;

  WalkState state;

  tf2::Transform support_to_last_;
  tf2::Transform support_to_next_;
  tf2::Transform left_in_world_;
  tf2::Transform right_in_world_;
};
}

#endif //BITBOTS_MOTION_BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_UTILS_H_