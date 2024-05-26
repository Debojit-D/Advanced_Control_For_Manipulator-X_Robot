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
/* This code was further edited by Debojit Das to incorporate Impedance Control (personal@debojit.in)*/

#ifndef advanced_robotics_control_TRAJECTORY_TRACKING_IMPEDANCE_CONTROL_CONTROLLER_H
#define advanced_robotics_control_TRAJECTORY_TRACKING_IMPEDANCE_CONTROL_CONTROLLER_H

#include <boost/scoped_ptr.hpp>
#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <urdf/model.h>
#include <std_msgs/Float64MultiArray.h> /// This line was include to publish the Torque Values
#include <kdl/chainjnttojacsolver.hpp> /// This line was included to publish the Jacobian
#include <kdl/chainfksolverpos_recursive.hpp>  // For forward kinematics
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/MultiArrayDimension.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>


namespace advanced_robotics_control
{
class TrajectoryTrackingImpedanceControlController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::EffortJointInterface>
{
 public:
  bool init(hardware_interface::RobotHW* robot_hardware,
            ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  // Joint handle
  hardware_interface::EffortJointInterface* effort_joint_interface_;
  std::vector<hardware_interface::JointHandle> effort_joint_handles_;
  std::vector<std::string> joint_names_;
  std::string root_name_;
  std::string tip_name_;

  double x_target_start = 0.0000;
  double x_target_end = 0.2500;
  double x_step = 0.0001;
  bool trajectory_active = true;

  // KDL
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::JntArray q_;
  KDL::JntArray q_dot_;
  KDL::JntArray tau_;
  KDL::Vector grav_;
  KDL::JntArray G_;

  // Simplified Impedance Control Parameter 
  KDL::Vector x_d_;  // Desired position of the end effector
  KDL::Vector x_o_;  // Desired oreintation of the end effector frame
  KDL::Vector x_ddot_;  // Desired position of the end effector
  KDL::Vector K_trans_;    // Stiffness vector for x, y, z directions
  KDL::Vector K_rot_; 
  KDL::Vector Kd_lin_;
  KDL::Vector Kd_ang_;  

  // KDL solver
  boost::scoped_ptr<KDL::ChainDynParam> MCG_solver_;
  
  KDL::ChainJntToJacSolver* jnt_to_jac_solver_;
  KDL::ChainFkSolverPos_recursive* fk_solver_;
  
  ros::Publisher end_effector_pos_publisher_;
  ros::Publisher torque_publisher_;  /// Publisher for the torques
  ros::Publisher jacobian_publisher_; /// Publisher for the Jacobian
  ros::Publisher joint_velocity_publisher_;  // Publisher for joint velocities
  ros::Publisher velocity_publisher_;
  ros::Publisher end_eff_velocity_publisher_;
  ros::Publisher end_effector_orient_euler_publisher_;
};
}  // namespace open_manipulator_controllers
#endif  // OPEN_MANIPULATOR_CONTROLLERS_IMPEDANCE_CONTROL_CONTROLLER_H