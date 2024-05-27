#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from geometry_msgs.msg import Pose, Point, Quaternion

def send_pose():
    rospy.init_node('set_kinematics_pose_client')
    
    rospy.wait_for_service('/goal_task_space_path')
    try:
        set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
        
        # Define the pose
        target_pose = Pose()
        #target_pose.position = Point(x=0.2, y=0.2, z=0.2)
        #target_pose.position = Point(x=0.1  , y=0.0, z=0.25)
        target_pose.position = Point(x=0.2  , y=-0.2, z=0.2)
        target_pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
        
        # Define the kinematics pose
        kinematics_pose = SetKinematicsPoseRequest()
        kinematics_pose.kinematics_pose.pose = target_pose
        kinematics_pose.planning_group = "manipulator"
        kinematics_pose.end_effector_name = "gripper"
        kinematics_pose.path_time = 1
        #kinematics_pose.max_accelerations_scaling_factor = 0.0
        #kinematics_pose.max_velocity_scaling_factor = 0.0
        #kinematics_pose.tolerance = 0.0

        # Call the service
        response = set_pose(kinematics_pose)
        
        if response.is_planned:
            rospy.loginfo("Pose successfully sent to the robot.")
        else:
            rospy.loginfo("Failed to send pose.")
            
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    send_pose()
