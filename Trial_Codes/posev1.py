#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np

def interpolate(start, end, steps):
    """ Linear interpolation between start and end points """
    return [start + (end - start) * float(i) / (steps - 1) for i in range(steps)]

def send_pose(start_pos, end_pos, resolution, pause_time):
    rospy.init_node('set_kinematics_pose_client')
    
    rospy.wait_for_service('/goal_task_space_path')
    try:
        set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
        
        x_points = interpolate(start_pos.x, end_pos.x, resolution)
        y_points = interpolate(start_pos.y, end_pos.y, resolution)
        z_points = interpolate(start_pos.z, end_pos.z, resolution)
        
        for x, y, z in zip(x_points, y_points, z_points):
            target_pose = Pose()
            target_pose.position = Point(x=x, y=y, z=z)
            target_pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
            
            kinematics_pose = SetKinematicsPoseRequest()
            kinematics_pose.kinematics_pose.pose = target_pose
            kinematics_pose.planning_group = "manipulator"
            kinematics_pose.end_effector_name = "gripper"
            kinematics_pose.path_time = 1  # Adjust this for speed control
            
            response = set_pose(kinematics_pose)
            
            if response.is_planned:
                rospy.loginfo("Pose successfully sent to the robot.")
            else:
                rospy.loginfo("Failed to send pose.")
            
            rospy.sleep(pause_time)  # Adjust the sleep time to control speed of motion
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    start_point = Point(x=0.1, y=0.0, z=0.25)
    end_point = Point(x=0.2, y=0.0, z=0.25)
    resolution = 5  # Number of points including start and end
    pause_time = 1  # Time in seconds between points #### In this code I can't get the pause time below 1 
    send_pose(start_point, end_point, resolution, pause_time)
