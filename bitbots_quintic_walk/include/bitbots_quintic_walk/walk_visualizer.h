#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_VISUALIZER_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_VISUALIZER_H_

#include <ros/ros.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "bitbots_splines/abstract_visualizer.h"
#include <bitbots_quintic_walk/WalkDebug.h>
#include <bitbots_quintic_walk/WalkEngineDebug.h>
#include <moveit_msgs/RobotState.h>
#include <bitbots_quintic_walk/walk_utils.h>
#include <moveit/robot_state/robot_state.h>
//#include "tf2/transform_datatypes.h"
//#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace bitbots_quintic_walk {
class WalkVisualizer : public bitbots_splines::AbstractVisualizer {
 public:
  explicit WalkVisualizer();
  explicit WalkVisualizer(std::shared_ptr<ros::NodeHandle> nh);

  void publishArrowMarker(std::string name_space,
                          std::string frame,
                          geometry_msgs::Pose pose,
                          float r,
                          float g,
                          float b,
                          float a);

  void publishEngineDebug(WalkResponse response);
  void publishIKDebug(WalkResponse response,
                                      robot_state::RobotStatePtr goal_state,
                                      robot_state::RobotStatePtr current_state,
                                      tf2::Transform trunk_to_support_foot_goal,
                                      tf2::Transform trunk_to_flying_foot_goal);
  void publishWalkMarkers(WalkResponse response);

 private:

  int marker_id_;

  ros::Publisher pub_debug_;
  ros::Publisher pub_engine_debug_;
  ros::Publisher pub_debug_marker_;
};
}

#endif //BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_VISUALIZER_H_