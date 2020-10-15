#include "bitbots_quintic_walk/walk_node.h"
#include "swri_profiler/profiler.h"

#include <memory>
#include <iostream>

namespace bitbots_quintic_walk {

WalkNode::WalkNode(const std::string ns) :
    robot_model_loader_(ns + "robot_description", false),
    stabilizer_(ns),
    walk_engine_(ns) {
  nh_ = ros::NodeHandle(ns);
  SWRI_PROFILE(ros::this_node::getName());
  // init variables
  robot_state_ = humanoid_league_msgs::RobotControlState::CONTROLLABLE;
  current_request_.linear_orders = {0, 0, 0};
  current_request_.angular_z = 0;
  current_trunk_fused_pitch_ = 0;
  current_trunk_fused_roll_ = 0;
  current_fly_pressure_ = 0;
  current_fly_effort_ = 0;

  current_trunk_pitch_ = 0;
  current_trunk_roll_ = 0;
  cop_right_x_ = 0;
  cop_right_y_ = 0;
  cop_left_x_ = 0;
  cop_left_y_ = 0;
  roll_vel_ = 0;
  pitch_vel_ = 0;

  // read config
  nh_.param<double>("engine_frequency", engine_frequency_, 100.0);
  nh_.param<bool>("simulation_active", simulation_active_, false);
  nh_.param<bool>("walking/node/publish_odom_tf", publish_odom_tf_, false);

  /* init publisher and subscriber */
  pub_controller_command_ = nh_.advertise<bitbots_msgs::JointCommand>("walking_motor_goals", 1);
  pub_odometry_ = nh_.advertise<nav_msgs::Odometry>("walk_engine_odometry", 1);
  pub_support_ = nh_.advertise<std_msgs::Char>("walk_support_state", 1, true);
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &WalkNode::cmdVelCb, this,
                               ros::TransportHints().tcpNoDelay());
  robot_state_sub_ = nh_.subscribe("robot_state", 1, &WalkNode::robotStateCb, this,
                                   ros::TransportHints().tcpNoDelay());
  joint_state_sub_ =
      nh_.subscribe("joint_states", 1, &WalkNode::jointStateCb, this, ros::TransportHints().tcpNoDelay());
  kick_sub_ = nh_.subscribe("kick", 1, &WalkNode::kickCb, this, ros::TransportHints().tcpNoDelay());
  imu_sub_ = nh_.subscribe("imu/data", 1, &WalkNode::imuCb, this, ros::TransportHints().tcpNoDelay());
  pressure_sub_left_ = nh_.subscribe("foot_pressure_left/filtered", 1, &WalkNode::pressureLeftCb, this,
                                     ros::TransportHints().tcpNoDelay());
  pressure_sub_right_ = nh_.subscribe("foot_pressure_right/filtered", 1, &WalkNode::pressureRightCb, this,
                                      ros::TransportHints().tcpNoDelay());

  cop_l_sub_ = nh_.subscribe("cop_l", 1, &WalkNode::copLeftCb, this, ros::TransportHints().tcpNoDelay());
  cop_r_sub_ = nh_.subscribe("cop_r", 1, &WalkNode::copRightCb, this, ros::TransportHints().tcpNoDelay());

  //load MoveIt! model
  robot_model_loader_.loadKinematicsSolvers(std::make_shared<kinematics_plugin_loader::KinematicsPluginLoader>());
  kinematic_model_ = robot_model_loader_.getModel();
  if (!kinematic_model_) {
    ROS_FATAL("No robot model loaded, killing quintic walk.");
    exit(1);
  }
  //stabilizer_.setRobotModel(kinematic_model_);
  ik_.init(kinematic_model_);
  visualizer_.init(kinematic_model_);

  current_state_.reset(new robot_state::RobotState(kinematic_model_));
  current_state_->setToDefaultValues();

  first_run_ = true;

  // initialize dynamic-reconfigure
  dyn_reconf_server_ =
      new dynamic_reconfigure::Server<bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig>(ros::NodeHandle(ns +
          "/walking/node"));
  dynamic_reconfigure::Server<bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig>::CallbackType f;
  f = boost::bind(&bitbots_quintic_walk::WalkNode::reconfCallback, this, _1, _2);
  dyn_reconf_server_->setCallback(f);

  pid_foot_pos_x_.init(ros::NodeHandle(ns + "/walking/pid_foot_pos_x"), false);
  pid_foot_pos_y_.init(ros::NodeHandle(ns + "/walking/pid_foot_pos_y"), false);

  pid_left_x_.init(ros::NodeHandle(ns + "/walking/pid_ankle_left_pitch"), false);
  pid_left_y_.init(ros::NodeHandle(ns + "/walking/pid_ankle_left_roll"), false);
  pid_right_x_.init(ros::NodeHandle(ns + "/walking/pid_ankle_right_pitch"), false);
  pid_right_y_.init(ros::NodeHandle(ns + "/walking/pid_ankle_right_roll"), false);
  pid_hip_pitch_.init(ros::NodeHandle(ns + "/walking/pid_hip_pitch"), false);
  pid_hip_roll_.init(ros::NodeHandle(ns + "/walking/pid_hip_roll"), false);

  pid_foot_pos_x_.reset();
  pid_foot_pos_y_.reset();

  pid_left_x_.reset();
  pid_left_y_.reset();
  pid_right_x_.reset();
  pid_right_y_.reset();
  pid_hip_pitch_.reset();
  pid_hip_roll_.reset();

  // this has to be done to prevent strange initilization bugs
  walk_engine_ = WalkEngine(ns);
}

void WalkNode::run() {
  int odom_counter = 0;
  walk_engine_.reset();

  while (ros::ok()) {
    ros::Rate loop_rate(engine_frequency_);
    double dt = getTimeDelta();

    if (robot_state_ == humanoid_league_msgs::RobotControlState::FALLING) {
      // the robot fell, we have to reset everything and do nothing else
      walk_engine_.reset();

      pid_foot_pos_x_.reset();
      pid_foot_pos_y_.reset();

      pid_left_x_.reset();
      pid_left_y_.reset();
      pid_right_x_.reset();
      pid_right_y_.reset();
      pid_hip_pitch_.reset();
      pid_hip_roll_.reset();

      stabilizer_.reset();
    } else {
      // we don't want to walk, even if we have orders, if we are not in the right state
      /* Our robots will soon^TM be able to sit down and stand up autonomously, when sitting down the motors are
       * off but will turn on automatically which is why MOTOR_OFF is a valid walkable state. */
      // TODO Figure out a better way than having integration knowledge that HCM will play an animation to stand up
      current_request_.walkable_state = robot_state_ == humanoid_league_msgs::RobotControlState::CONTROLLABLE ||
          robot_state_ == humanoid_league_msgs::RobotControlState::WALKING ||
          robot_state_ == humanoid_league_msgs::RobotControlState::MOTOR_OFF;

      // perform all the actual calculations
      bitbots_msgs::JointCommand joint_goals = step(dt);

      // only publish goals if we are not idle
      if (walk_engine_.getState() != WalkState::IDLE) {
        pub_controller_command_.publish(joint_goals);

        // publish current support state
        std_msgs::Char support_state;
        if (walk_engine_.isDoubleSupport()) {
          support_state.data = 'd';
        } else if (walk_engine_.isLeftSupport()) {
          support_state.data = 'l';
        } else {
          support_state.data = 'r';
        }
        // publish if foot changed
        if (current_support_foot_ != support_state.data) {
          pub_support_.publish(support_state);
          current_support_foot_ = support_state.data;
        }

        // publish debug information
        if (debug_active_) {
          visualizer_.publishIKDebug(current_stabilized_response_, current_state_, motor_goals_);
          visualizer_.publishWalkMarkers(current_stabilized_response_);
          visualizer_.publishEngineDebug(current_response_);
        }
      }

      // publish odometry
      odom_counter++;
      if (odom_counter > odom_pub_factor_) {
        publishOdometry(current_response_);
        odom_counter = 0;
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}

bitbots_msgs::JointCommand WalkNode::step(double dt) {
  SWRI_PROFILE("step");
  // PID control on foot position. take previous goal orientation and compute difference with actual orientation
  Eigen::Quaterniond goal_orientation_eigen;
  tf2::convert(current_response_.support_foot_to_trunk.getRotation(), goal_orientation_eigen);
  rot_conv::FusedAngles goal_fused = rot_conv::FusedFromQuat(goal_orientation_eigen);
  WalkRequest request(current_request_);
  request.linear_orders[0] +=
      pid_foot_pos_x_.computeCommand(goal_fused.fusedPitch - current_trunk_fused_pitch_, ros::Duration(dt));
  request.linear_orders[1] +=
      pid_foot_pos_y_.computeCommand(goal_fused.fusedRoll - current_trunk_fused_roll_, ros::Duration(dt));

  // update walk engine response
  walk_engine_.setGoals(request);
  checkPhaseRestAndReset();
  current_response_ = walk_engine_.update(dt);

  // only calculate joint goals from this if the engine is not idle
  current_response_.current_fused_roll = current_trunk_fused_roll_;
  current_response_.current_fused_pitch = current_trunk_fused_pitch_;

  current_response_.current_pitch = current_trunk_pitch_;
  current_response_.current_roll = current_trunk_roll_;
  current_response_.roll_vel = roll_vel_;
  current_response_.pitch_vel = pitch_vel_;
  if (walk_engine_.isLeftSupport()) {
    current_response_.sup_cop_x = cop_left_x_;
    current_response_.sup_cop_y = cop_left_y_;
  } else {
    current_response_.sup_cop_x = cop_right_x_;
    current_response_.sup_cop_y = cop_right_y_;
  }

  // get stabilized goals from stabilizer
  current_stabilized_response_ = stabilizer_.stabilize(current_response_, ros::Duration(dt));

  // compute motor goals from IK
  motor_goals_ = ik_.calculate(current_stabilized_response_);

  double goal_pitch, goal_roll, goal_yaw;
  tf2::Matrix3x3(current_stabilized_response_.support_foot_to_trunk.getRotation())
      .getRPY(goal_roll, goal_pitch, goal_yaw);
  double hip_pitch_correction =
      pid_hip_pitch_.computeCommand(goal_pitch - current_response_.current_pitch, ros::Duration(dt));
  double hip_roll_correction =
      pid_hip_roll_.computeCommand(goal_roll - current_response_.current_roll, ros::Duration(dt));

  for (int i = 0; i < motor_goals_.first.size(); ++i) {
    if (motor_goals_.first.at(i) == "LAnklePitch") {
      motor_goals_.second.at(i) += pid_left_x_.computeCommand(cop_left_x_, ros::Duration(dt));
    } else if (motor_goals_.first.at(i) == "RAnklePitch") {
      motor_goals_.second.at(i) += pid_right_x_.computeCommand(cop_right_x_, ros::Duration(dt));
    } else if (motor_goals_.first.at(i) == "LAnkleRoll") {
      motor_goals_.second.at(i) += pid_left_y_.computeCommand(cop_left_y_, ros::Duration(dt));
    } else if (motor_goals_.first.at(i) == "RAnkleRoll") {
      motor_goals_.second.at(i) += pid_right_y_.computeCommand(cop_right_y_, ros::Duration(dt));
    } else if (motor_goals_.first.at(i) == "RHipPitch") {
      // pitch servos have inverted sign, therefore substract
      motor_goals_.second.at(i) -= hip_pitch_correction;
    } else if (motor_goals_.first.at(i) == "LHipPitch") {
      motor_goals_.second.at(i) += hip_pitch_correction;
    } else if (motor_goals_.first.at(i) == "LHipRoll" || motor_goals_.first.at(i) == "RHipRoll") {
      motor_goals_.second.at(i) += hip_roll_correction;
    }
  }
  // change to joint command message type
  bitbots_msgs::JointCommand command;
  command.header.stamp = ros::Time::now();

  /*
   * Since our JointGoals type is a vector of strings
   *  combined with a vector of numbers (motor name -> target position)
   *  and bitbots_msgs::JointCommand needs both vectors as well,
   *  we can just assign them
   */
  command.joint_names = motor_goals_.first;
  command.positions = motor_goals_.second;

  /* And because we are setting position goals and not movement goals, these vectors are set to -1.0*/
  std::vector<double> vels(motor_goals_.first.size(), -1.0);
  std::vector<double> accs(motor_goals_.first.size(), -1.0);
  std::vector<double> pwms(motor_goals_.first.size(), -1.0);
  command.velocities = vels;
  command.accelerations = accs;
  command.max_currents = pwms;

  return command;
}

double WalkNode::getTimeDelta() {
  // compute time delta depended if we are currently in simulation or reality
  double dt;
  double current_ros_time = ros::Time::now().toSec();
  dt = current_ros_time - last_ros_update_time_;
  if (dt == 0) {
    ROS_WARN("dt was 0");
    dt = 0.001;
  }
  last_ros_update_time_ = current_ros_time;

  // time is wrong when we run it for the first time
  if (first_run_) {
    first_run_ = false;
    dt = 0.0001;
  }
  return dt;
}

void WalkNode::reset() {
  walk_engine_.reset();
  stabilizer_.reset();
}

bitbots_msgs::JointCommand WalkNode::step(double dt,
                                          const geometry_msgs::Twist &cmdvel_msg,
                                          const sensor_msgs::Imu &imu_msg,
                                          const sensor_msgs::JointState &jointstate_msg,
                                          const bitbots_msgs::FootPressure &pressure_left,
                                          const bitbots_msgs::FootPressure &pressure_right) {
  // method for python interface. take all messages as parameters instead of using ROS
  cmdVelCb(cmdvel_msg);
  imuCb(imu_msg);
  jointStateCb(jointstate_msg);
  pressureLeftCb(pressure_left);
  pressureRightCb(pressure_right);
  // we don't use external robot state
  current_request_.walkable_state = true;
  // update walk engine response
  bitbots_msgs::JointCommand joint_goals = step(dt);
  return joint_goals;
}

geometry_msgs::Pose WalkNode::get_left_foot_pose() {
  robot_state::RobotStatePtr goal_state = ik_.get_goal_state();
  geometry_msgs::Pose pose;
  tf2::convert(goal_state->getGlobalLinkTransform("l_sole"), pose);
  return pose;
}

void WalkNode::cmdVelCb(const geometry_msgs::Twist msg) {
  // we use only 3 values from the twist messages, as the robot is not capable of jumping or spinning around its
  // other axis.

  // the engine expects orders in [m] not [m/s]. We have to compute by dividing by step frequency which is a double step
  // factor 2 since the order distance is only for a single step, not double step
  double factor = (1.0 / (walk_engine_.getFreq())) / 2.0;
  // the sideward movement only does one step per double step, therefore we need to multiply it by 2
  current_request_.linear_orders = {msg.linear.x * factor, msg.linear.y * factor * 2, msg.linear.z * factor};
  current_request_.angular_z = msg.angular.z * factor;

  // the orders should not extend beyond a maximal step size
  for (int i = 0; i < 3; i++) {
    current_request_.linear_orders[i] =
        std::max(std::min(current_request_.linear_orders[i], max_step_linear_[i]), max_step_linear_[i] * -1);
  }
  current_request_.angular_z =
      std::max(std::min(current_request_.angular_z, max_step_angular_), max_step_angular_ * -1);
  // translational orders (x+y) should not exceed combined limit. scale if necessary
  if (max_step_xy_ != 0) {
    double scaling_factor = (current_request_.linear_orders[0] + current_request_.linear_orders[1]) / max_step_xy_;
    for (int i = 0; i < 2; i++) {
      current_request_.linear_orders[i] = current_request_.linear_orders[i] / std::max(scaling_factor, 1.0);
    }
  }

  // warn user that speed was limited
  if (msg.linear.x * factor != current_request_.linear_orders[0] ||
      msg.linear.y * factor != current_request_.linear_orders[1] / 2 ||
      msg.linear.z * factor != current_request_.linear_orders[2] ||
      msg.angular.z * factor != current_request_.angular_z) {
    ROS_WARN(
        "Speed command was x: %.2f y: %.2f z: %.2f angular: %.2f xy: %.2f but maximum is x: %.2f y: %.2f z: %.2f angular: %.2f xy: %.2f",
        msg.linear.x,
        msg.linear.y,
        msg.linear.z,
        msg.angular.z,
        msg.linear.x + msg.linear.y,
        max_step_linear_[0] / factor,
        max_step_linear_[1] / factor / 2,
        max_step_linear_[2] / factor,
        max_step_angular_ / factor,
        max_step_xy_ / factor);
  }
}

void WalkNode::imuCb(const sensor_msgs::Imu &msg) {
  // the incoming geometry_msgs::Quaternion is transformed to a tf2::Quaternion
  tf2::Quaternion quat;
  tf2::convert(msg.orientation, quat);
  // the tf2::Quaternion has a method to access roll pitch and yaw
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  current_trunk_pitch_ = pitch;
  current_trunk_roll_ = roll;

  Eigen::Quaterniond imu_orientation_eigen;
  tf2::convert(quat, imu_orientation_eigen);
  rot_conv::FusedAngles imu_fused = rot_conv::FusedFromQuat(imu_orientation_eigen);

  current_trunk_fused_pitch_ = imu_fused.fusedPitch;
  current_trunk_fused_roll_ = imu_fused.fusedRoll;

  // get angular velocities
  roll_vel_ = msg.angular_velocity.x;
  pitch_vel_ = msg.angular_velocity.y;

  if (imu_active_) {
    // compute the pitch offset to the currently wanted pitch of the engine
    double wanted_pitch = walk_engine_.getWantedTrunkPitch();

    double pitch_delta = pitch - wanted_pitch;
    if (abs(roll) > imu_roll_threshold_ || abs(pitch_delta) > imu_pitch_threshold_ ||
        abs(pitch_vel_) > imu_pitch_vel_threshold_ || abs(roll_vel_) > imu_roll_vel_threshold_) {
      walk_engine_.requestPause();
      if (abs(roll) > imu_roll_threshold_) {
        ROS_WARN("imu roll angle stop");
      } else if (abs(pitch_delta) > imu_pitch_threshold_) {
        ROS_WARN("imu pitch angle stop");
      } else if (abs(pitch_vel_) > imu_pitch_vel_threshold_) {
        ROS_WARN("imu roll vel stop");
      } else {
        ROS_WARN("imu pitch vel stop");
      }
    }
  }
}

void WalkNode::pressureLeftCb(const bitbots_msgs::FootPressure msg) {
  // only check if this foot is not the current support foot
  if (!walk_engine_.isLeftSupport()) {
    current_fly_pressure_ = msg.left_back + msg.left_front + msg.right_back + msg.right_front;
  }
}

void WalkNode::pressureRightCb(const bitbots_msgs::FootPressure msg) {
  // only check if this foot is not the current support foot
  if (walk_engine_.isLeftSupport()) {
    current_fly_pressure_ = msg.left_back + msg.left_front + msg.right_back + msg.right_front;
  }
}

void WalkNode::checkPhaseRestAndReset() {
  /**
   * This method checks if the foot made contact to the ground and ends the step earlier by resetting the phase ("phase reset")
     or lets the robot rest until it makes ground contact ("phase rest").
   */
  // phase has to be far enough (almost at end of step) so that the foot has already lifted from the ground
  // otherwise we will always do phase reset in the beginning of the step
  double phase = walk_engine_.getPhase();
  double phase_reset_phase = walk_engine_.getPhaseResetPhase();

  if ((phase > phase_reset_phase && phase < 0.5) || (phase > 0.5 + phase_reset_phase)) {
    // check if we want to perform a phase reset
    if (pressure_phase_reset_active_ && current_fly_pressure_ > ground_min_pressure_) {
      // reset phase by using pressure sensors
      ROS_WARN("Phase resetted by pressure!");
      walk_engine_.endStep();
    } else if (effort_phase_reset_active_ && current_fly_effort_ > joint_min_effort_) {
      // reset phase by using joint efforts
      ROS_WARN("Phase resetted by effort!");
      walk_engine_.endStep();
    }
  }
}

void WalkNode::copLeftCb(geometry_msgs::PointStamped msg) {
  // Since CoP is only published as something else than 0 if the foot is on the ground, we don't have to check
  // if this is the current support foot, or if it is double support.
  // The CoP should not go to the outside edge of the foot. This is the left side for the left foot (y is positive).
  // The inside edge is okay, since it is necessary to shift the CoM dynamically between feet.
  // To prevent falling to the front and back, the x position of the CoP is also taken into account.
  if (cop_stop_active_ &&
      (msg.point.y > cop_y_threshold_ || abs(msg.point.x) > cop_x_threshold_)) {
    walk_engine_.requestPause();
  }
  cop_left_x_ = msg.point.x;
  cop_left_y_ = msg.point.y;
}

void WalkNode::copRightCb(geometry_msgs::PointStamped msg) {
  // The CoP should not go to the outside edge of the foot. This is the right side for the right foot (y is negative).
  if (cop_stop_active_ &&
      (msg.point.y < -1 * cop_y_threshold_ || abs(msg.point.x) > cop_x_threshold_)) {
    walk_engine_.requestPause();
  }

  cop_right_x_ = msg.point.x;
  cop_right_y_ = msg.point.y;
}

void WalkNode::robotStateCb(const humanoid_league_msgs::RobotControlState msg) {
  robot_state_ = msg.state;
}

void WalkNode::jointStateCb(const sensor_msgs::JointState &msg) {
  std::vector<std::string> names = msg.name;
  std::vector<double> goals = msg.position;
  for (int i = 0; i < names.size(); i++) {
    // besides its name, this method only changes a single joint position...
    current_state_->setJointPositions(names[i], &goals[i]);
  }

  // compute the effort that is currently on the flying leg to check if it has ground contact
  // only if we have the necessary data
  if (msg.effort.size() == msg.name.size()) {
    double effort_sum = 0;
    const std::vector<std::string>
        &fly_joint_names = (walk_engine_.isLeftSupport()) ? ik_.getRightLegJointNames() : ik_.getLeftLegJointNames();
    for (int i = 0; i < names.size(); i++) {
      // add effort on this joint to sum, if it is part of the flying leg
      if (std::find(fly_joint_names.begin(), fly_joint_names.end(), names[i]) != fly_joint_names.end()) {
        effort_sum = effort_sum + abs(msg.effort[i]);
      }
    }
    current_fly_effort_ = effort_sum;
  } else {
    ROS_WARN_ONCE("Joint states have no effort information. Phase reset based on this will not work.");
  }
}

void WalkNode::kickCb(const std_msgs::BoolConstPtr &msg) {
  walk_engine_.requestKick(msg->data);
}

void WalkNode::reconfCallback(bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig &config, uint32_t level) {
  params_ = config;

  ik_.setIKTimeout(config.ik_timeout);

  debug_active_ = config.debug_active;
  engine_frequency_ = config.engine_freq;
  odom_pub_factor_ = config.odom_pub_factor;

  max_step_linear_[0] = config.max_step_x;
  max_step_linear_[1] = config.max_step_y;
  max_step_linear_[2] = config.max_step_z;
  max_step_angular_ = config.max_step_angular;
  max_step_xy_ = config.max_step_xy;

  imu_active_ = config.imu_active;
  imu_pitch_threshold_ = config.imu_pitch_threshold;
  imu_roll_threshold_ = config.imu_roll_threshold;
  imu_pitch_vel_threshold_ = config.imu_pitch_vel_threshold;
  imu_roll_vel_threshold_ = config.imu_roll_vel_threshold;

  pressure_phase_reset_active_ = config.pressure_phase_reset_active;
  effort_phase_reset_active_ = config.effort_phase_reset_active;
  ground_min_pressure_ = config.ground_min_pressure;
  joint_min_effort_ = config.joint_min_effort;
  // phase rest can only work if one phase resetting method is active
  if (effort_phase_reset_active_ || pressure_phase_reset_active_) {
    walk_engine_.setPhaseRest(config.phase_rest_active);
  } else {
    walk_engine_.setPhaseRest(false);
  }

  params_.pause_duration = config.pause_duration;
  walk_engine_.setPauseDuration(params_.pause_duration);
}

void WalkNode::publishOdometry(WalkResponse response) {
  // odometry to trunk is transform to support foot * transform from support to trunk
  tf2::Transform support_foot_tf;
  if (walk_engine_.isLeftSupport()) {
    support_foot_tf = walk_engine_.getLeft();
  } else {
    support_foot_tf = walk_engine_.getRight();
  }

  tf2::Transform odom_to_trunk = support_foot_tf * response.support_foot_to_trunk;
  tf2::Vector3 pos = odom_to_trunk.getOrigin();
  geometry_msgs::Quaternion quat_msg;
  tf2::convert(odom_to_trunk.getRotation().normalize(), quat_msg);

  ros::Time current_time = ros::Time::now();

  // send the odometry as message
  odom_msg_.header.stamp = current_time;
  odom_msg_.header.frame_id = "odom";
  odom_msg_.child_frame_id = "base_link";
  odom_msg_.pose.pose.position.x = pos[0];
  odom_msg_.pose.pose.position.y = pos[1];
  odom_msg_.pose.pose.position.z = pos[2];

  odom_msg_.pose.pose.orientation = quat_msg;
  geometry_msgs::Twist twist;

  twist.linear.x = current_request_.linear_orders.x() * walk_engine_.getFreq() * 2;
  twist.linear.y = current_request_.linear_orders.y() * walk_engine_.getFreq() * 2;
  twist.linear.z = current_request_.linear_orders.z() * walk_engine_.getFreq() * 2;
  twist.angular.z = current_request_.angular_z * walk_engine_.getFreq() * 2;

  odom_msg_.twist.twist = twist;
  pub_odometry_.publish(odom_msg_);

  if (publish_odom_tf_) {
    odom_trans_ = geometry_msgs::TransformStamped();
    odom_trans_.header.stamp = current_time;
    odom_trans_.header.frame_id = "odom";
    odom_trans_.child_frame_id = "base_link";

    odom_trans_.transform.translation.x = pos[0];
    odom_trans_.transform.translation.y = pos[1];
    odom_trans_.transform.translation.z = pos[2];
    odom_trans_.transform.rotation = quat_msg;

    //send the transform
    odom_broadcaster_.sendTransform(odom_trans_);
  }
}

void WalkNode::initializeEngine() {
  walk_engine_.reset();
}

WalkEngine *WalkNode::getEngine() {
  return &walk_engine_;
}

} // namespace bitbots_quintic_walk

int main(int argc, char **argv) {
  ros::init(argc, argv, "walking");
  // init node
  bitbots_quintic_walk::WalkNode node("");

  // run the node
  node.initializeEngine();
  node.run();
}
