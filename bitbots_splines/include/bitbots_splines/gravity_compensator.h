#pragma once

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/revolute_joint_model.h>
#include <ros/ros.h>

/*
 * A class for effort based gravity compensation
 */
class GravityCompensator {
 public:
  GravityCompensator(moveit::core::RobotModelConstPtr &robot_model) {
    efforts_.resize(robot_model->getVariableCount());
    // Initialize link masses and centers
    for (auto *link : robot_model->getLinkModels()) {
      link_masses_[link] = PointMass();
      if (auto urdf_link = robot_model->getURDF()->getLink(link->getName())) {
        if (urdf_link->inertial && urdf_link->inertial->mass > 0) {
          link_masses_[link].mass = urdf_link->inertial->mass;
          link_masses_[link].center.x() = urdf_link->inertial->origin.position.x;
          link_masses_[link].center.y() = urdf_link->inertial->origin.position.y;
          link_masses_[link].center.z() = urdf_link->inertial->origin.position.z;
        }
      }
    }
  }

  /*
   * Compensate gravity
   *
   * @param state the current robot state
   * @param contacts A vector of weighted links contacting the ground
   */
  void compensateGravity(moveit::core::RobotStatePtr state,
                         const std::vector<std::pair<std::string, double>> &contacts) {
    std::fill(efforts_.begin(), efforts_.end(), 0);
    for (auto &contact: contacts) {
      std::string link_name = contact.first;
      double contact_weight = contact.second;
      const moveit::core::LinkModel *contact_link = state->getRobotModel()->getLinkModel(link_name);
      addLink(state, contact_link, nullptr, contact_weight);
    }
    for (size_t i = 0; i < state->getVariableCount(); i++) {
      // P controller to change the position
      state->setVariablePosition(i, state->getVariablePosition(i) + 0.02 * efforts_[i]);
    }
    for (auto *joint: state->getRobotModel()->getJointModels()) {
      if (joint->getType() == moveit::core::JointModel::REVOLUTE) {
        //ROS_WARN_STREAM(joint->getName() << ": " << efforts_[joint->getFirstVariableIndex()]);
      }
    }
  }

 private:
  /* A struct defining a center of mass */
  class PointMass {
   public:
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double mass = 0;
    PointMass &operator+=(const PointMass &other) {
      center = (center * mass + other.center * other.mass) / (mass + other.mass);
      mass += other.mass;
      return *this;
    }
  };

  /* A map of links to their mass centers */
  std::unordered_map<const moveit::core::LinkModel *, PointMass> link_masses_;
  Eigen::Vector3d gravity_ = Eigen::Vector3d(0, 0, -9.81);
  std::vector<double> efforts_;

  /*
   * Recursively add all joints to a tree starting from the contact point.
   * The contact link is the root of the tree, the connected links are the children and so on.
   * Thereby, every joint (i.e. every edge of the tree) supports all the links below it in the tree.
   * Efforts on a joint are calculated when all child elements have been added.
   * @param state the current robot state
   * @param link the link whose children we are adding
   * @param coming_joint the joint above the current link in the tree, null if link is the contact link
   * @param the weight of the effort
   * @returns the center of mass of all links below and including the current link
   */
  PointMass addLink(const moveit::core::RobotStatePtr state,
                    const moveit::core::LinkModel *link,
                    const moveit::core::JointModel *coming_joint,
                    double weight) {
    /* link_list is a list of the links below the current one in the tree */
    PointMass link_list;

    std::vector<const moveit::core::JointModel *> connected_joints = link->getChildJointModels();
    connected_joints.push_back(link->getParentJointModel());
    // Iterate over all joints of the current link
    for (auto *joint : connected_joints) {
      // Skip links already in tree
      if (joint == coming_joint) continue;

      // List of all the links below current joints, calculated recursively
      PointMass links_below_current_joint;
      if (link == joint->getChildLinkModel()) {
        // Child already in tree, add parent
        if (joint->getParentLinkModel()) {
          // parent link is null for base joint
          links_below_current_joint = addLink(state, joint->getParentLinkModel(), joint, weight);
        }
      } else {
        // Parent already in tree, add child
        links_below_current_joint = addLink(state, joint->getChildLinkModel(), joint, weight);
      }
      // All child links are added, calculate effort on current joint now if their mass is > 0
      if (links_below_current_joint.mass > 0) {
        double effort_on_joint = calculateEffortForJoint(state, joint, links_below_current_joint);
        efforts_[joint->getFirstVariableIndex()] += effort_on_joint * weight;
        // Add child links of current joint to list of links since they are also below the current link
        link_list += links_below_current_joint;
      }
    }
    // Finally add the current link to the list if its mass is larger than 0
    if (link_masses_[link].mass > 0) {
      link_list += link_masses_[link];
    }
    return link_list;
  }

  /*
   * calculate the effort on a given joint
   * @param state the current robot state
   * @param joint the joint of that the effort will be calculated
   * @param link_below_current_joint the links that should be considered for effort calculation
   * @returns the calculated effort
   */
  double calculateEffortForJoint(const robot_state::RobotStatePtr state,
                                 const moveit::core::JointModel *joint,
                                 PointMass links_below_current_joint) {
    Eigen::Isometry3d joint_transform;

    // Get rotation of joint
    joint->computeTransform(state->getVariablePositions() + joint->getFirstVariableIndex(), joint_transform);

    // parent link is null for the root joint which is in the coordinate system center, use Identity Transform then
    Eigen::Isometry3d transform_to_parent =
        joint->getParentLinkModel() ? state->getGlobalLinkTransform(joint->getParentLinkModel())
                                    : Eigen::Isometry3d::Identity();

    // Get global transform to joint
    // child->getJointOriginTransform returns the transform from the parent to the joint
    joint_transform = transform_to_parent * joint->getChildLinkModel()->getJointOriginTransform() * joint_transform;

    // We compensate only revolute joints
    if (auto *j = dynamic_cast<const moveit::core::RevoluteJointModel *>(joint)) {
      std::string name = j->getName();
      // Get joint axis in global frame
      auto joint_axis = joint_transform.rotation() * j->getAxis();
      // calculate effort on joint axis
      return joint_axis.cross(gravity_).dot(links_below_current_joint.center - joint_transform.translation()) * links_below_current_joint.mass;
    }
    return 0;
  }
};
