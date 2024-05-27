#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from geometry_msgs.msg import Pose, Point, Quaternion

def send_pose_series():
    rospy.init_node('set_kinematics_pose_series_client')
    
    rospy.wait_for_service('/goal_task_space_path')
    set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
    
    # Define a series of poses
    poses = [
        Point(x=0.2, y=-0.2, z=0.2),
        Point(x=0.2, y=-0.19, z=0.2),
        Point(x=0.2, y=-0.18, z=0.2),
        Point(x=0.2, y=-0.17, z=0.2),
        Point(x=0.2, y=-0.16, z=0.2),
        Point(x=0.2, y=-0.15, z=0.2),
        Point(x=0.2, y=-0.14, z=0.2),
        Point(x=0.2, y=-0.13, z=0.2),
        Point(x=0.2, y=-0.12, z=0.2),
        Point(x=0.2, y=-0.11, z=0.2),
        Point(x=0.2, y=-0.10, z=0.2),
        Point(x=0.2, y=-0.09, z=0.2),
        Point(x=0.2, y=-0.08, z=0.2),
        Point(x=0.2, y=-0.07, z=0.2),
        Point(x=0.2, y=-0.06, z=0.2),
    ]
    orientation = Quaternion(x=0, y=0, z=0, w=1)  # constant orientation

    try:
        rate = rospy.Rate(10)
        for point in poses:
            # Define the kinematics pose
            kinematics_pose = SetKinematicsPoseRequest()
            kinematics_pose.kinematics_pose.pose.position = point
            kinematics_pose.kinematics_pose.pose.orientation = orientation
            kinematics_pose.planning_group = "manipulator"
            kinematics_pose.end_effector_name = "gripper"
            kinematics_pose.path_time = 0.1  # duration to reach the pose

            # Call the service
            response = set_pose(kinematics_pose)
            if response.is_planned:
                rospy.loginfo("Pose successfully sent to the robot.")
            else:
                rospy.loginfo("Failed to send pose.")
                
            rate.sleep()

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    try:
        send_pose_series()
    except rospy.ROSInterruptException:
        pass