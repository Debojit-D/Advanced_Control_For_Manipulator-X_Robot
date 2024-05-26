/*******************************************************************************
* Impedance Control for Robotic Manipulation
* 
* Description:
* This implementation of Impedance Control is specifically tailored for a 4 DOF robotic 
* manipulator, focusing on achieving compliant behavior through force feedback control. 
* The code structure and computation methodologies were initially inspired by and adapted 
* from existing gravity compensation strategies employed previously in the gravity compensation
* structure provided for Manipulator X by Robotis

*
* Author: Debojit Das - Undergraduate Researcher, IIT Gandhinagar Robotics Lab
* 
* Based on prior work in Gravity Compensation by:
* Ryan Shim, ROBOTIS CO., LTD.
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


#include "advanced_robotics_control/impedance_control_controller.h"

namespace advanced_robotics_control
{
bool ImpedanceControlController::init(
    hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
  // Joint interface initialization
  effort_joint_interface_ =
      robot_hardware->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface_ == nullptr)
  {
    ROS_ERROR(
        "[ImpedanceControlController] Could not get effort joint interface "
        "from hardware!");
    return false;
  }

  // Initialize desired position (x_d_) and stiffness (K_trans)
  x_d_ = KDL::Vector(0.1747, 0.0000, 0.2000);  // This line defines the desired postion of the end effector of the robot. 
  x_o_ = KDL::Vector(0.0000, 0.9500, 0.0000);  // This line defines the desired orientation of the end effector of the robot.
  K_trans_ = KDL::Vector(20, 20, 20);  // Stiffness in x, y, z directions
  K_rot_ = KDL::Vector(1.0, 0.8 , 1.0); // Stiffness in roll, pitch and yaw
  // If you face difficulties to find the coordinates of the desired position 
  // you can directly run the gravity compensation there is a publisher that publishes the 
  // end effector position
  Kd_lin_ = KDL::Vector(0.5, 2.5, 2); // Translational Damping in x,y,z direction
  Kd_ang_ = KDL::Vector(0.020, 0.020, 0.020); // Rotational Damping Coefficients 

  // Joint handle
  if (!node_handle.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("[ImpedanceControlController] Could not parse joint names");
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
          "[ImpedanceControlController] Could not get joint handle: "
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
    ROS_ERROR("[ImpedanceControlController] Could not parse urdf file");
    return false;
  }
  if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
  {
    ROS_ERROR("[ImpedanceControlController] Could not construct kdl tree");
    return false;
  }
  if (!node_handle.getParam("root_link", root_name_))
  {
    ROS_ERROR("[ImpedanceControlController] Could not find root link name");
    return false;
  }
  if (!node_handle.getParam("tip_link", tip_name_))
  {
    ROS_ERROR("[ImpedanceControlController] Could not find tip link name");
    return false;
  }
  if (!kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_))
  {
    ROS_ERROR(
        "[ImpedanceControlController] Could not get KDL chain from tree");
    return false;
  }

  // After setting up the kdl_chain_
  jnt_to_jac_solver_ = new KDL::ChainJntToJacSolver(kdl_chain_);
  jacobian_publisher_ = node_handle.advertise<std_msgs::Float64MultiArray>("jacobian_values", 10);

  // Initialize the forward kinematics solver after setting up the kdl_chain_
  fk_solver_ = new KDL::ChainFkSolverPos_recursive(kdl_chain_);
  end_effector_pos_publisher_ = node_handle.advertise<geometry_msgs::Point>("end_effector_position", 10);
  velocity_publisher_ = node_handle.advertise<std_msgs::Float64MultiArray>("joint_velocities", 10);
  end_eff_velocity_publisher_ = node_handle.advertise<std_msgs::Float64MultiArray>("end_effector_velocities", 10);
  end_effector_orient_euler_publisher_ = node_handle.advertise<geometry_msgs::Vector3>("end_effector_orientation", 10);

  // Resize the variables
  q_.resize(kdl_chain_.getNrOfJoints());
  tau_.resize(kdl_chain_.getNrOfJoints());
  G_.resize(kdl_chain_.getNrOfJoints());

  // Gravity torque
  KDL::Vector g(0.0, 0.0, -9.81); // Change it how ou are orienting the robot and defining reference frame.
  grav_ = g;
  MCG_solver_.reset(new KDL::ChainDynParam(kdl_chain_, grav_));

  return true;
}

void ImpedanceControlController::starting(const ros::Time& time) {}

void ImpedanceControlController::update(const ros::Time& time,
                                           const ros::Duration& period)
{
    /*This section is to get the joint positions of the robot*/
    for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++)
    {
        q_(i) = effort_joint_handles_[i].getPosition();
    }

    /*This section is for calculating the Jacobian and Publishing it*/
    KDL::Jacobian jacobian(kdl_chain_.getNrOfJoints());
    if (jnt_to_jac_solver_->JntToJac(q_, jacobian) >= 0)
    {
      std_msgs::Float64MultiArray jacobian_msg;
      jacobian_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      jacobian_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());

      // Set up dimensions for the Jacobian matrix
      jacobian_msg.layout.dim[0].label = "rows";
      jacobian_msg.layout.dim[0].size = jacobian.rows();
      jacobian_msg.layout.dim[0].stride = jacobian.rows() * jacobian.columns();
      jacobian_msg.layout.dim[1].label = "columns";
      jacobian_msg.layout.dim[1].size = jacobian.columns();
      jacobian_msg.layout.dim[1].stride = jacobian.columns();

    // Reserve space for all elements
    jacobian_msg.data.reserve(jacobian.rows() * jacobian.columns());

    // Populate the data
    for (unsigned int i = 0; i < jacobian.rows(); ++i) {
        for (unsigned int j = 0; j < jacobian.columns(); ++j) {
            jacobian_msg.data.push_back(jacobian(i, j));
        }
    }

    // Publish the Jacobian matrix
    jacobian_publisher_.publish(jacobian_msg);
    }

    /*This section is to get the end effector coordinates of the robot and publish it*/
    // Compute forward kinematics for the end effector
    KDL::Frame end_effector_frame;  // This will store the resultant pose of the end effector
    geometry_msgs::Vector3 euler_angles;
    if(fk_solver_->JntToCart(q_, end_effector_frame) >= 0)
    {
        geometry_msgs::Point end_effector_pos;
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

    // Calculate rotational error
    double error_roll = x_o_.x() - euler_angles.x;
    double error_pitch = x_o_.y() - euler_angles.y;
    double error_yaw = x_o_.z() - euler_angles.z;

    // Normalize angles to be within the range of -π to π, if necessary
    error_roll = fmod(error_roll + M_PI, 2 * M_PI) - M_PI;
    error_pitch = fmod(error_pitch + M_PI, 2 * M_PI) - M_PI;
    error_yaw = fmod(error_yaw + M_PI, 2 * M_PI) - M_PI;


    /*This section is to get the joint veclocities of the robot and publish it*/
    // New: Fetch and publish velocities
    std_msgs::Float64MultiArray velocity_msg;
    velocity_msg.data.clear();
    for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++)
    {
        double velocity = effort_joint_handles_[i].getVelocity();  // If your handle supports it
        velocity_msg.data.push_back(velocity);
    }
    // Publish the velocities
    velocity_publisher_.publish(velocity_msg);

    
    Eigen::VectorXd joint_velocities(velocity_msg.data.size());
    for (size_t i = 0; i < velocity_msg.data.size(); ++i) {
        joint_velocities[i] = velocity_msg.data[i];
    }


    /*This section is to get the end effector velocity of the robot and publish it*/
    // Calculate the end-effector velocity in Cartesian space
    Eigen::VectorXd end_effector_velocity = jacobian.data * joint_velocities;

    velocity_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    velocity_msg.layout.dim[0].size = end_effector_velocity.size();
    velocity_msg.layout.dim[0].stride = 1;  // Each element is one double
    velocity_msg.layout.dim[0].label = "End Effector Velocity"; // You can label the dimension as needed

    // Copy the Eigen vector to the ROS message
    velocity_msg.data.resize(end_effector_velocity.size());
    for (int i = 0; i < end_effector_velocity.size(); ++i) {
        velocity_msg.data[i] = end_effector_velocity[i];
    }

    // Publish the message
    end_eff_velocity_publisher_.publish(velocity_msg);

    /*This section is to get gravity compensation term of the Impedance Control*/
    // Gravity compensation
    KDL::JntArray tau_gravity(kdl_chain_.getNrOfJoints());
    MCG_solver_->JntToGravity(q_, tau_gravity);

    /*This section is to get stiffness term of the Impedance Control*/
    // Calculate the error in Cartesian coordinates
    KDL::Vector error_trans = x_d_ - end_effector_frame.p;

    // Compute the spring force in Cartesian space
    KDL::Vector spring_force_trans(error_trans.x() * K_trans_.x(), error_trans.y() * K_trans_.y(), error_trans.z() * K_trans_.z());
    KDL::Vector spring_force_rot(K_rot_.x() * error_roll, K_rot_.y() * error_pitch, K_rot_.z() * error_yaw);
    
    // Convert spring force into joint torques using Jacobian transpose
    Eigen::VectorXd spring_force_vec(6); // Assuming 6 DOF: 3 translations + 3 rotations
    
    Eigen::MatrixXd jacobian_transpose = jacobian.data.transpose();
    spring_force_vec << spring_force_trans.x(), spring_force_trans.y(), spring_force_trans.z(),
                    spring_force_rot.x(), spring_force_rot.y(), spring_force_rot.z();

    Eigen::VectorXd tau_spring_vec = jacobian_transpose * spring_force_vec;


    // /*This section is to get damping term of the Impedance Control*/
    // Compute the error velocity (e_dot) in Cartesian space
    // Since x_ddot_ is zero, e_dot is just the negative of the translational part of the end-effector velocity
    Eigen::VectorXd e_dot(6);
    e_dot << -end_effector_velocity.head(3), -end_effector_velocity.tail(3); // Includes both translation and rotation

    // Compute the damping force in Cartesian space
    // Apply damping only to translational components and leave rotational components as zero
    Eigen::VectorXd damping_force(6);
    damping_force << Kd_lin_.x() * e_dot[0], Kd_lin_.y() * e_dot[1], Kd_lin_.z() * e_dot[2],
                 Kd_ang_.x() * e_dot[3], Kd_ang_.y() * e_dot[4], Kd_ang_.z() * e_dot[5];

    Eigen::VectorXd tau_damping_vec = jacobian_transpose * damping_force;


    // Final Impedance control equation
    KDL::JntArray tau_total(kdl_chain_.getNrOfJoints());
    for (int i = 0; i < tau_spring_vec.size(); i++) {
        tau_total(i) = tau_gravity(i) + tau_spring_vec(i) + tau_damping_vec(i);
    }

    // Publish torques
    std_msgs::Float64MultiArray torque_msg;
    torque_msg.data.clear();
    for (size_t i = 0; i < tau_total.rows(); i++) {
        torque_msg.data.push_back(tau_total(i));
    }
    torque_publisher_.publish(torque_msg);

    // Command torques to hardware
    for (size_t i = 0; i < tau_total.rows(); i++) {
        effort_joint_handles_[i].setCommand(tau_total(i));
    }
}

void ImpedanceControlController::stopping(const ros::Time& time) {}
}  // namespace open_manipulator_controllers

PLUGINLIB_EXPORT_CLASS(
    advanced_robotics_control::ImpedanceControlController,
    controller_interface::ControllerBase)

