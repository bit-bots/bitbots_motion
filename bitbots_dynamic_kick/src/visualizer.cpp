#include "bitbots_dynamic_kick/visualizer.h"

namespace bitbots_dynamic_kick {

Visualizer::Visualizer(const std::string& base_topic) :
    base_topic_(base_topic),
    params_() {
  /* make sure base_topic_ has consistent scheme */
  if (base_topic.compare(base_topic.size() - 1, 1, "/")!=0) {
    base_topic_ += "/";
  }

  /* create necessary publishers */
  goal_publisher_ = node_handle_.advertise<visualization_msgs::Marker>(base_topic_ + "received_goal",
      /* queue_size */ 5, /* latch */ true);
  spline_publisher_ = node_handle_.advertise<nav_msgs::Path>(base_topic_ + "flying_foot_spline",
      /* queue_size */ 5, /* latch */ true);
  windup_publisher_ = node_handle_.advertise<visualization_msgs::Marker>(base_topic_ + "kick_windup_point",
      /* queue_size */ 5, /* latch */ true);
  stabilizing_publisher_ = node_handle_.advertise<visualization_msgs::Marker>(base_topic_ + "kick_stabilizing_point",
      /* queue_size */ 5, /* latch */ true);

  node_handle_.getParam("/debug_active", param_debug_active_);
}

void Visualizer::setParams(VisualizationParams params) {
  params_ = params;
}

void Visualizer::displayFlyingSplines(const bitbots_splines::Trajectories &splines, const std::string &support_foot_frame) {
  if (!isEnabled())
    return;

  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = support_foot_frame;

  for (int i = 0; i < splines.size() * params_.spline_smoothness; i++) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = path.header.stamp;
    pose.header.frame_id = support_foot_frame;
    pose.pose.position.x = splines.get("pos_x").pos((float) i/(float) params_.spline_smoothness);
    pose.pose.position.y = splines.get("pos_y").pos((float) i/(float) params_.spline_smoothness);
    pose.pose.position.z = splines.get("pos_z").pos((float) i/(float) params_.spline_smoothness);
    pose.pose.orientation.w = 1;

    path.poses.push_back(pose);
  }

  spline_publisher_.publish(path);
}

void Visualizer::displayReceivedGoal(const bitbots_msgs::KickGoalConstPtr &goal) {
  if (!isEnabled())
    return;

  visualization_msgs::Marker
      marker = getMarker({goal->ball_position.x, goal->ball_position.y, goal->ball_position.z}, goal->header.frame_id);

  marker.ns = marker_ns_;
  marker.id = MarkerIDs::RECEIVED_GOAL;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.header.stamp = goal->header.stamp;
  marker.pose.orientation = goal->kick_direction;
  marker.scale.x = 0.08 + (goal->kick_speed/3);
  marker.color.r = 1;

  goal_publisher_.publish(marker);
}

void Visualizer::displayWindupPoint(const tf2::Vector3 &kick_windup_point, const std::string &support_foot_frame) {
  if (!isEnabled())
    return;

  visualization_msgs::Marker marker = getMarker(kick_windup_point, support_foot_frame);

  marker.ns = marker_ns_;
  marker.id = MarkerIDs::RECEIVED_GOAL;
  marker.color.g = 1;

  windup_publisher_.publish(marker);
}

void Visualizer::displayStabilizingPoint(const tf2::Vector3 &kick_windup_point,
                                         const std::string &support_foot_frame) {
  if (!isEnabled())
    return;

  visualization_msgs::Marker marker = getMarker(kick_windup_point, support_foot_frame);

  marker.ns = marker_ns_;
  marker.id = MarkerIDs::KICK_STABILIZING_POINT;
  marker.color.g = 1;

  stabilizing_publisher_.publish(marker);
}

bool Visualizer::isEnabled() {
  return params_.force_enable || param_debug_active_;
}

}