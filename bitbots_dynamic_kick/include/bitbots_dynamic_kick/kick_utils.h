#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_UTILS_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_UTILS_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Transform.h>

namespace bitbots_dynamic_kick {

struct KickPositions {
  bool is_left_kick = true;
  tf2::Transform trunk_pose;
  tf2::Transform flying_foot_pose;

  /**
   * Should center-of-pressure support be used to stabilize this specific goal.
   *
   * If false, this indicates to any center-of-pressure stabilizer that they should not be used despite being
   * active in general.
   */
  bool cop_support_point = false;
};

struct KickGoals {
  std_msgs::Header header;
  geometry_msgs::Vector3 ball_position;
  geometry_msgs::Quaternion kick_direction;
  double kick_speed = 0;
  geometry_msgs::Pose r_foot_pose;
  geometry_msgs::Pose l_foot_pose;
};

}

#endif //BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_UTILS_H_
