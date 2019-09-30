#include "bitbots_dynup/dynup_node.h"

namespace bitbots_dynup {

DynUpNode::DynUpNode() :
    server_(node_handle_, "dynup", boost::bind(&DynUpNode::executeCb, this, _1), false),
    listener_(tf_buffer_) {
  joint_goal_publisher_ = node_handle_.advertise<bitbots_msgs::JointCommand>("animation_motor_goals", 1);
  server_.start();
}

void DynUpNode::reconfigureCallback(bitbots_dynup::DynUpConfig &config, uint32_t level) {
  engine_rate_ = config.engine_rate;

  DynUpParams params = DynUpParams();
  //TODO actually set parameters like
  //params.leg_min_length = config.leg_min_length;
  params.foot_distance = config.foot_distance;
  params.rise_time = config.rise_time;
  params.trunk_x = config.trunk_x;
  params.trunk_height = config.trunk_height;
  params.trunk_pitch = config.trunk_pitch;

  engine_.setParams(params);

  engine_.stabilizer.useMinimalDisplacement(config.minimal_displacement);
  engine_.stabilizer.useStabilizing(config.stabilizing);
  engine_.stabilizer.setStabilizingWeight(config.stabilizing_weight);
}

void DynUpNode::executeCb(const bitbots_msgs::DynUpGoalConstPtr &goal) {
  // TODO: maybe switch to goal callback to be able to reject goals properly
  ROS_INFO("Accepted new goal");
  engine_.reset();

  if (std::optional<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> poses = getCurrentPoses()) {
    engine_.start(true, poses->first, poses->second); //todo we are currently only getting up from squad
    loopEngine();
    bitbots_msgs::DynUpResult r;
    r.successful = true;
    server_.setSucceeded(r);
  } else {
    ROS_ERROR_STREAM("Could not determine foot positions! Aborting standup.");
    bitbots_msgs::DynUpResult r;
    r.successful = false;
    server_.setAborted(r);
  }
}

void DynUpNode::loopEngine() {
  /* Do the loop as long as nothing cancels it */
  while (server_.isActive() && !server_.isPreemptRequested()) {
    if (std::optional<JointGoals> goals = engine_.tick(1.0 / engine_rate_)) {
      // TODO: add counter for failed ticks
      bitbots_msgs::DynUpFeedback feedback;
      feedback.percent_done = engine_.getPercentDone();
      server_.publishFeedback(feedback);
      publishGoals(goals.value());

      if (feedback.percent_done == 100) {
        break;
      }
    }

    /* Let ROS do some important work of its own and sleep afterwards */
    ros::spinOnce();
    ros::Rate loop_rate(engine_rate_);
    loop_rate.sleep();
  }
}

std::optional<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> DynUpNode::getCurrentPoses() {
  // return left foot pose and trunk pose relative to right foot
  ros::Time time = ros::Time::now();

  /* Construct zero-positions for both poses in their respective local frames */
  geometry_msgs::PoseStamped l_foot_origin, trunk_origin;
  l_foot_origin.header.frame_id = "l_sole";
  l_foot_origin.pose.orientation.w = 1;
  l_foot_origin.header.stamp = time;

  trunk_origin.header.frame_id = "torso";
  trunk_origin.pose.orientation.w = 1;
  trunk_origin.header.stamp = time;

  /* Transform both poses into the right foot frame */
  geometry_msgs::PoseStamped l_foot_transformed, trunk_transformed;
  try {
    tf_buffer_.transform(l_foot_origin, l_foot_transformed, "r_sole",
                         ros::Duration(0.2)); // TODO lookup thrown exceptions in internet and catch
    tf_buffer_.transform(trunk_origin, trunk_transformed, "r_sole", ros::Duration(0.2));
    return std::pair(l_foot_transformed.pose, trunk_transformed.pose);
  } catch (tf2::TransformException&) {
    return std::nullopt;
  }

}

void DynUpNode::publishGoals(const JointGoals &goals) {
  /* Construct JointCommand message */
  bitbots_msgs::JointCommand command;
  command.header.stamp = ros::Time::now();

  /*
   * Since our JointGoals type is a vector of strings
   *  combined with a vector of numbers (motor name -> target position)
   *  and bitbots_msgs::JointCommand needs both vectors as well,
   *  we can just assign them
   */
  command.joint_names = goals.first;
  command.positions = goals.second;

  /* And because we are setting position goals and not movement goals, these vectors are set to -1.0*/
  std::vector<double> vels(goals.first.size(), -1.0);
  std::vector<double> accs(goals.first.size(), -1.0);
  std::vector<double> pwms(goals.first.size(), -1.0);
  command.velocities = vels;
  command.accelerations = accs;
  command.max_currents = pwms;

  joint_goal_publisher_.publish(command);
}
void DynUpNode::publishSupportFoot(bool is_left_dyn_up) {
  // Currently not implemented
}

}

int main(int argc, char *argv[]) {
  /* Setup ROS node */
  ros::init(argc, argv, "dynup");
  bitbots_dynup::DynUpNode node;

  /* Setup dynamic_reconfigure */
  dynamic_reconfigure::Server<bitbots_dynup::DynUpConfig> dyn_reconf_server;
  dynamic_reconfigure::Server<bitbots_dynup::DynUpConfig>::CallbackType f;
  f = boost::bind(&bitbots_dynup::DynUpNode::reconfigureCallback, &node, _1, _2);
  dyn_reconf_server.setCallback(f);

  ROS_INFO("Initialized DynUp and waiting for actions");
  ros::spin();
}
