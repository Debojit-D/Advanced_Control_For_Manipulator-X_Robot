/*******************************************************************************
* Copyright 2020 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Ryan Shim */

#include "advanced_robotics_control/gravity_compensation_controller.h"

namespace advanced_robotics_control
{
bool GravityCompensationController::init(
    hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
  // Joint interface
  effort_joint_interface_ =
      robot_hardware->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface_ == nullptr)
  {
    ROS_ERROR(
        "[GravityCompensationController] Could not get effort joint interface "
        "from hardware!");
    return false;
  }

  // Joint handle
  if (!node_handle.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("[GravityCompensationController] Could not parse joint names");
    return false;
  }
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    try
    {
      effort_joint_handles_.push_back(
          effort_joint_interface_->getHandle(joint_names_[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM(
          "[GravityCompensationController] Could not get joint handle: "
          << e.what());
      return false;
    }
  }

  /// Initialize the publisher for torques
  torque_publisher_ = node_handle.advertise<std_msgs::Float64MultiArray>("torque_values", 10);

  // KDL
  urdf::Model urdf;
  if (!urdf.initParam("robot_description"))
  {
    ROS_ERROR("[GravityCompensationController] Could not parse urdf file");
    return false;
  }
  if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
  {
    ROS_ERROR("[GravityCompensationController] Could not construct kdl tree");
    return false;
  }
  if (!node_handle.getParam("root_link", root_name_))
  {
    ROS_ERROR("[GravityCompensationController] Could not find root link name");
    return false;
  }
  if (!node_handle.getParam("tip_link", tip_name_))
  {
    ROS_ERROR("[GravityCompensationController] Could not find tip link name");
    return false;
  }
  if (!kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_))
  {
    ROS_ERROR(
        "[GravityCompensationController] Could not get KDL chain from tree");
    return false;
  }

  // Initialize the forward kinematics solver after setting up the kdl_chain_
  fk_solver_ = new KDL::ChainFkSolverPos_recursive(kdl_chain_);
  end_effector_pos_publisher_ = node_handle.advertise<geometry_msgs::Point>("end_effector_position", 10);
  end_effector_orient_euler_publisher_ = node_handle.advertise<geometry_msgs::Vector3>("end_effector_orientation", 10);

  // Resize the variables
  q_.resize(kdl_chain_.getNrOfJoints());
  tau_.resize(kdl_chain_.getNrOfJoints());
  G_.resize(kdl_chain_.getNrOfJoints());

  // Gravity torque
  KDL::Vector g(0.0, 0.0, -9.81);
  grav_ = g;
  MCG_solver_.reset(new KDL::ChainDynParam(kdl_chain_, grav_));

  return true;
}

void GravityCompensationController::starting(const ros::Time& time) {}

void GravityCompensationController::update(const ros::Time& time,
                                           const ros::Duration& period)
{
    // Get the current joint positions
    for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++)
    {
        q_(i) = effort_joint_handles_[i].getPosition();
    }

    // Compute the gravity torque
    MCG_solver_->JntToGravity(q_, G_);

    // Prepare the message to publish torques
    std_msgs::Float64MultiArray torque_msg;
    torque_msg.data.clear();  // Clear previous data if any

    for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++)
    {
        tau_(i) = G_(i);  // Set the computed gravity compensation torques
        torque_msg.data.push_back(tau_(i));  // Add torque to message
    }

    // Publish the torques
    torque_publisher_.publish(torque_msg);

    // Compute forward kinematics for the end effector
    KDL::Frame end_effector_frame;  // This will store the resultant pose of the end effector
    geometry_msgs::Point end_effector_pos;
    geometry_msgs::Vector3 euler_angles;
    if(fk_solver_->JntToCart(q_, end_effector_frame) >= 0)
    {
        end_effector_pos.x = end_effector_frame.p.x();
        end_effector_pos.y = end_effector_frame.p.y();
        end_effector_pos.z = end_effector_frame.p.z();

        // Publish the end effector position
        end_effector_pos_publisher_.publish(end_effector_pos);

        // New: Publish the end-effector orientation as Euler angles
        double roll, pitch, yaw;
        end_effector_frame.M.GetRPY(roll, pitch, yaw);
        euler_angles.x = roll;
        euler_angles.y = pitch;
        euler_angles.z = yaw;
        end_effector_orient_euler_publisher_.publish(euler_angles);
    }

    // Set effort commands to the hardware
    for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++)
    {
        effort_joint_handles_[i].setCommand(tau_(i));
    }
}


void GravityCompensationController::stopping(const ros::Time& time) {}
}  // namespace advanced_robotics_control

PLUGINLIB_EXPORT_CLASS(
    advanced_robotics_control::GravityCompensationController,
    controller_interface::ControllerBase)
