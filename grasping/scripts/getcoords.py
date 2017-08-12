#!/usr/bin/env python
import rospy
from std_msgs.msg import String
 
import baxter_interface
from geometry_msgs.msg import PoseStamped


def get_real_coords(slot, qr_pose ):

	d_x=5
	d_y=5
	n=7	
	gr= MoveGroupInterface("right_arm", "base")

 	position = qr_pose.position
    quat = qr_pose.orientation
    rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))
    rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))

    qr_x_0=position.x  
    qr_y_0=position.y

    #calculate origin of board wrt qr

    o_x=qr_x_0+qr_x
    o_y=qr_y_0+qr_y


    col=slot%n
    row=int(slot/n)

    x_new=(d_x/2)+d_x*col
    y_new=(d_y/2)+d_y*row

    x_pose=x_new+o_x
    y_pose=y_new+o_y

   
    # Move left arm to pick object and pick object
    goal_pose = PoseStamped() 
    goal_pose.header.frame_id = "base"
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.pose.position.x = x_pose
    goal_pose.pose.position.y = y_pose
    goal_pose.pose.position.z = qr_pose.position.z
    goal_pose.pose.orientation.x = qr_pose.orientation.x
    goal_pose.pose.orientation.y = qr_pose.orientation.y
    goal_pose.pose.orientation.z = qr_pose.orientation.z
    goal_pose.pose.orientation.w = qr_pose.orientation.w
    gr.moveToPose(goal_pose, "right_gripper", plan_only=False)

