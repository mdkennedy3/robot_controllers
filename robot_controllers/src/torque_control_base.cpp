/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2017, Fetch Robotics Inc.
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

// Author: Michael Ferguson

#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <robot_controllers/torque_control_base.h>

PLUGINLIB_EXPORT_CLASS(robot_controllers::TorqueControllerBase, robot_controllers::Controller)

namespace robot_controllers
{

TorqueControllerBase::TorqueControllerBase() :
    initialized_(false),
    safety_scaling_(1.0)
{
  theta_ = 0.0;

  odom_.pose.pose.orientation.z = 0.0;
  odom_.pose.pose.orientation.w = 1.0;

  last_sent_x_ = desired_x_ = 0.0;
  last_sent_r_ = desired_r_ = 0.0;

  left_last_timestamp_ = right_last_timestamp_ = 0.0;
  last_command_time_ = last_update_ = ros::Time(0.0);
}

int TorqueControllerBase::init(ros::NodeHandle& nh, ControllerManager* manager)
{
  // We absolutely need access to the controller manager
  if (!manager)
  {
    ROS_ERROR_NAMED("BaseController", "No controller manager available.");
    initialized_ = false;
    return -1;
  }

  Controller::init(nh, manager);
  manager_ = manager;

  // Initialize joints
  std::string l_name, r_name;
  nh.param<std::string>("l_wheel_joint", l_name, "l_wheel_joint");
  nh.param<std::string>("r_wheel_joint", r_name, "r_wheel_joint");
  left_ = manager_->getJointHandle(l_name);
  right_ = manager_->getJointHandle(r_name);
  if (left_ == NULL || right_ == NULL)
  {
    ROS_ERROR_NAMED("BaseController", "Cannot get wheel joints.");
    initialized_ = false;
    return -1;
  }
  left_last_position_ = left_->getPosition();
  right_last_position_ = right_->getPosition();
  last_update_ = ros::Time::now();

  // Initialize joints vector
  joint_names_.clear();
  joint_names_.push_back(l_name);
  joint_names_.push_back(r_name);

  joints_.clear();
  joints_.push_back(manager_->getJointHandle(l_name));
  joints_.push_back(manager_->getJointHandle(r_name));

  command_efforts_.clear();
  command_efforts_.push_back(0.0);
  command_efforts_.push_back(0.0);

  last_command_time_ = last_update_ = ros::Time(0.0);

  // Get base parameters
  nh.param<double>("track_width", track_width_, 0.37476);
  nh.param<double>("radians_per_meter", radians_per_meter_, 16.5289);

  // If using an external correction (such as robot_pose_ekf or graft)
  // we should not publish the TF frame from base->odom
  nh.param<bool>("publish_tf", publish_tf_, true);

  // The pose in the odometry message is specified in terms of the odometry frame
  nh.param<std::string>("odometry_frame", odom_.header.frame_id, "odom");

  // The twist in the odometry message is specified in the coordinate frame of the base
  nh.param<std::string>("base_frame", odom_.child_frame_id, "base_link");

  // Get various thresholds below which we supress noise
  nh.param<double>("wheel_rotating_threshold", wheel_rotating_threshold_, 0.001);
  nh.param<double>("rotating_threshold", rotating_threshold_, 0.05);
  nh.param<double>("moving_threshold", moving_threshold_, 0.05);

  double t;
  nh.param<double>("timeout", t, 0.25);
  timeout_ = ros::Duration(t);

  // Subscribe to base commands
  command_sub_ = nh.subscribe<trajectory_msgs::JointTrajectory>("command", 1,
                    boost::bind(&TorqueControllerBase::command, this, _1));

  initialized_ = true;

  // Should we autostart?
  bool autostart;
  nh.param("autostart", autostart, false);
  if (autostart)
    manager->requestStart(getName());

  return 0;
}

bool TorqueControllerBase::start()
{
  if (!initialized_)
  {
    ROS_ERROR_NAMED("BaseController", "Unable to start, not initialized.");
    return false;
  }

  return true;
}

bool TorqueControllerBase::reset()
{
  // Reset command
  last_command_time_ = ros::Time(0);
  return true;
}

void TorqueControllerBase::update(const ros::Time& now, const ros::Duration& dt)
{
  if (!initialized_)
    return;  // should never really hit this

  // Copy command to local to reduce lock contention
  std::vector<double> command_efforts;
  ros::Time last_command_time;
  {
    boost::mutex::scoped_lock lock(mutex_);
    command_efforts = command_efforts_;
    last_command_time = last_command_time_;
  }

  size_t num_joints = joint_names_.size();

  // Get current velocities
  /*
  for (size_t i = 0; i < num_joints; ++i)
  {
    //positions_.q.data[i] = joints_[i]->getVelocity();
  }
  */

  bool command_active = (now - last_command_time) < timeout_;

  if (!command_active)
  {
    //TODO slowly get velocity to ramp to zero
    for (size_t i = 0; i < num_joints; ++i)
      joints_[i]->setEffort(0.0);
  }
  else
  {
    for (size_t i = 0; i < num_joints; ++i)
    {
      if(command_efforts[i] == std::numeric_limits<double>::max())
        joints_[i]->setEffort(0.0); //Set Zeros to the joints not commaneded
      else
        joints_[i]->setEffort(command_efforts[i]); //TODO limit effort to max/min
    }
  }

}

void TorqueControllerBase::command(const trajectory_msgs::JointTrajectory::ConstPtr& goal)
{
  // Need to initialize KDL structs
  if (!initialized_)
  {
    ROS_ERROR("TorqueControllerBase: Cannot accept goal, controller is not initialized.");
    return;
  }

  if (goal->points.empty())
  {
    // Stop
    ROS_INFO_STREAM_NAMED("TorqueControllerBase", "TorqueControllerBase no points");
    manager_->requestStop(getName());
    return;
  }

  size_t num_joints = joints_.size();

  // Find mapping of joint names into message joint names
  std::vector<size_t> mapping(num_joints, std::string::npos);
  for (size_t j = 0; j < num_joints; ++j)
  {
    for (size_t i = 0; i < goal->joint_names.size(); ++i)
    {
      if (joint_names_[j] == goal->joint_names[i])
      {
        mapping[j] = i;
        break;
      }
    }
  }

  std::vector<double> command_efforts(num_joints, std::numeric_limits<double>::max());
  for (size_t j = 0; j < num_joints; ++j)
  {
    if (0) ROS_INFO_STREAM_NAMED("TorqueControllerBase", "TorqueControllerBase point " << j << "->" << mapping[j]);

    if (mapping[j] != std::string::npos)
    {
      double effort = goal->points[0].effort[mapping[j]];
      if (!std::isfinite(effort)) effort = 0.0;
      command_efforts[j] = effort;
    }
  }

  ros::Time now(ros::Time::now());
  {
    boost::mutex::scoped_lock lock(mutex_);
    command_efforts_ = command_efforts;
    last_command_time_ = now;
  }

  // Try to start up
  if (manager_->requestStart(getName()) != 0)
  {
    ROS_ERROR("TorqueControllerBase: Cannot start, blocked by another controller.");
    return;
  }
}

std::vector<std::string> TorqueControllerBase::getCommandedNames()
{
  std::vector<std::string> names;
  if (left_)
    names.push_back(left_->getName());
  if (right_)
    names.push_back(right_->getName());
  return names;
}

std::vector<std::string> TorqueControllerBase::getClaimedNames()
{
  // Claimed == Commanded
  return getCommandedNames();
}

}  // namespace robot_controllers
