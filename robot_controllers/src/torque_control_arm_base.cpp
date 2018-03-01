/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Fetch Robotics Inc.
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Unbounded Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Michael Ferguson */

#include <pluginlib/class_list_macros.h>
#include <robot_controllers/torque_control_arm_base.h>

PLUGINLIB_EXPORT_CLASS(robot_controllers::TorqueControllerArmBase, robot_controllers::Controller)

namespace robot_controllers
{

int TorqueControllerArmBase::init(ros::NodeHandle& nh, ControllerManager* manager)
{
  Controller::init(nh, manager);
  manager_ = manager;

  // Load URDF
  urdf::Model model;
  if (!model.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse URDF");
    return -1;
  }

  // Load the tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Could not construct tree from URDF");
    return -1;
  }

  // Populate the Chain
  std::string root, tip;
  nh.param<std::string>("root", root, "torso_lift_link");
  nh.param<std::string>("tip", tip, "wrist_roll_link");
  if(!kdl_tree.getChain(root, tip, kdl_chain_))
  {
    ROS_ERROR("Could not construct chain from URDF");
    return -1;
  }

  kdl_chain_dynamics_.reset(new KDL::ChainDynParam(kdl_chain_, KDL::Vector(0,0,-9.81)));

  // Init positions
  positions_ = KDL::JntArrayVel(kdl_chain_.getNrOfJoints());
  KDL::SetToZero(positions_.q);
  KDL::SetToZero(positions_.qdot);

  // Init Joint Handles
  arm_joints_.clear();
  for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
  {
    KDL::Joint joint = kdl_chain_.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::None)
    {
      arm_joint_names_.push_back(joint.getName());
    }
    else
    {
      ROS_INFO_STREAM_NAMED("TorqueControllerArmBase", "Missing joint " << i);
    }
  }
  size_t arm_num_joints = arm_joint_names_.size();

  for (size_t i = 0; i < arm_num_joints; i++) {
    arm_joints_.push_back(manager_->getJointHandle(arm_joint_names_[i]));
  }

  /*
  for (size_t i = 0; i < arm_num_joints; i++) {
    ROS_ERROR("max effort %s %g", arm_joints_[i]->getName().c_str(), arm_joints_[i]->getEffortMax());
  }
  */

  arm_command_efforts_.clear();
  arm_smoothed_efforts_.clear();
  for (size_t i = 0; i < arm_num_joints; ++i)
  {
    arm_command_efforts_.push_back(0.0);
    arm_smoothed_efforts_.push_back(0.0);
  }
  gravity_compensation_factor_ = 1.0;


  //Init Base Joint handles
  std::string l_name, r_name;
  nh.param<std::string>("l_wheel_joint", l_name, "l_wheel_joint");
  nh.param<std::string>("r_wheel_joint", r_name, "r_wheel_joint");

  base_joint_names_.clear();
  base_joint_names_.push_back(l_name);
  base_joint_names_.push_back(r_name);

  base_joints_.clear();
  base_joints_.push_back(manager_->getJointHandle(l_name));
  base_joints_.push_back(manager_->getJointHandle(r_name));

  base_command_efforts_.clear();
  base_command_efforts_.push_back(0.0);
  base_command_efforts_.push_back(0.0);

  // Subscribe to command
  command_sub_ = nh.subscribe<trajectory_msgs::JointTrajectory>("command", 1,
                    boost::bind(&TorqueControllerArmBase::command, this, _1));
  last_command_time_ = ros::Time(0);

  initialized_ = true;

  ROS_INFO_STREAM_NAMED("TorqueControllerArmBase", "Inited TorqueControllerArmBase " << getName());

  // Should we autostart?
  bool autostart;
  nh.param("autostart", autostart, false);
  if (autostart)
    manager->requestStart(getName());

  return 0;
}

bool TorqueControllerArmBase::start()
{
  if (!initialized_)
    return false;
  return true;
}

void TorqueControllerArmBase::update(const ros::Time& now, const ros::Duration& dt)
{
  // Need to initialize KDL structs
  if (!initialized_)
    return;

  // Copy command to local to reduce lock contention
  std::vector<double> arm_command_efforts;
  std::vector<double> base_command_efforts;
  ros::Time last_command_time;
  {
    boost::mutex::scoped_lock lock(mutex_);
    arm_command_efforts = arm_command_efforts_;
    base_command_efforts = base_command_efforts_;
    last_command_time = last_command_time_;
  }

  size_t arm_num_joints = arm_joint_names_.size();
  size_t base_num_joints = base_joint_names_.size();

  // Get current positions
  for (size_t i = 0; i < arm_num_joints; ++i)
  {
    positions_.q.data[i] = arm_joints_[i]->getPosition();
  }

  // Do the gravity compensation
  KDL::JntArray torques(arm_num_joints);
  kdl_chain_dynamics_->JntToGravity(positions_.q, torques);

  bool command_active = (now - last_command_time) < ros::Duration(0.25);

  if (!command_active)
  {
    for (size_t i = 0; i < arm_num_joints; ++i)
      arm_joints_[i]->setEffort(torques.data[i]);

    for (size_t i = 0; i < base_num_joints; ++i)
      base_joints_[i]->setEffort(0.0);
  }
  else
  {
    for (size_t i = 0; i < arm_num_joints; ++i)
    {
      if(arm_command_efforts[i] == std::numeric_limits<double>::max())
        arm_joints_[i]->setEffort(torques.data[i]); //Get gravity comp values for joints not commaneded
      else
        arm_joints_[i]->setEffort(arm_command_efforts[i]); //TODO limit effort to max/min
    }

    for (size_t i = 0; i < base_num_joints; ++i)
    {
      if(base_command_efforts[i] == std::numeric_limits<double>::max())
        base_joints_[i]->setEffort(0.0); //Get gravity comp values for joints not commaneded
      else
        base_joints_[i]->setEffort(base_command_efforts[i]); //TODO limit effort to max/min
    }
  }
}

void TorqueControllerArmBase::command(const trajectory_msgs::JointTrajectory::ConstPtr& goal)
{
  // Need to initialize KDL structs
  if (!initialized_)
  {
    ROS_ERROR("TorqueControllerArmBase: Cannot accept goal, controller is not initialized.");
    return;
  }

  if (goal->points.empty())
  {
    // Stop
    ROS_INFO_STREAM_NAMED("TorqueControllerArmBase", "TorqueControllerArmBase no points");
    manager_->requestStop(getName());
    return;
  }

  size_t arm_num_joints = arm_joints_.size();

  // Find arm_mapping of joint names into message joint names
  std::vector<size_t> arm_mapping(arm_num_joints, std::string::npos);
  for (size_t j = 0; j < arm_num_joints; ++j)
  {
    for (size_t i = 0; i < goal->joint_names.size(); ++i)
    {
      if (arm_joint_names_[j] == goal->joint_names[i])
      {
        arm_mapping[j] = i;
        break;
      }
    }
  }

  std::vector<double> arm_command_efforts(arm_num_joints, std::numeric_limits<double>::max());
  for (size_t j = 0; j < arm_num_joints; ++j)
  {
    if (0) ROS_INFO_STREAM_NAMED("TorqueControllerArmBase", "TorqueControllerArmBase point " << j << "->" << arm_mapping[j]);

    if (arm_mapping[j] != std::string::npos)
    {
      double effort = goal->points[0].effort[arm_mapping[j]];
      if (!std::isfinite(effort)) effort = 0.0;
      arm_command_efforts[j] = effort;
    }
  }

  size_t base_num_joints = base_joints_.size();

  // Find base_mapping of joint names into message joint names
  std::vector<size_t> base_mapping(base_num_joints, std::string::npos);
  for (size_t j = 0; j < base_num_joints; ++j)
  {
    for (size_t i = 0; i < goal->joint_names.size(); ++i)
    {
      if (base_joint_names_[j] == goal->joint_names[i])
      {
        base_mapping[j] = i;
        break;
      }
    }
  }

  std::vector<double> base_command_efforts(base_num_joints, std::numeric_limits<double>::max());
  for (size_t j = 0; j < base_num_joints; ++j)
  {
    if (0) ROS_INFO_STREAM_NAMED("TorqueControllerArmBase", "TorqueControllerArmBase point " << j << "->" << base_mapping[j]);

    if (base_mapping[j] != std::string::npos)
    {
      double effort = goal->points[0].effort[base_mapping[j]];
      if (!std::isfinite(effort)) effort = 0.0;
      base_command_efforts[j] = effort;
    }
  }

  ros::Time now(ros::Time::now());
  {
    boost::mutex::scoped_lock lock(mutex_);
    arm_command_efforts_ = arm_command_efforts;
    base_command_efforts_ = base_command_efforts;
    last_command_time_ = now;
  }

  // Try to start up
  if (manager_->requestStart(getName()) != 0)
  {
    ROS_ERROR("TorqueControllerArmBase: Cannot start, blocked by another controller.");
    return;
  }
}


std::vector<std::string> TorqueControllerArmBase::getCommandedNames()
{
  std::vector<std::string> names;
  if (initialized_)
  {
    for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
      if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
        names.push_back(kdl_chain_.getSegment(i).getJoint().getName());
  }
  return names;
}

std::vector<std::string> TorqueControllerArmBase::getClaimedNames()
{
  // We don't claim anything
  std::vector<std::string> names;
  return names;
}

}  // namespace robot_controllers
