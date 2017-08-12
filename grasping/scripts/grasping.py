#!/usr/bin/env python
# license removed for brevity

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

#"""
def  main():
    #rospy.init_node('grasping', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("right_arm")
    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)

    print "============ Waiting for RVIZ..."
    #rospy.sleep(10)
    print "============ Starting tutorial "

    print "============ Reference frame: %s" % group.get_planning_frame()

    print "============ Reference frame: %s" % group.get_end_effector_link()

    print "============ Robot Groups:"
    print robot.get_group_names()

    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"

    print "============ Generating plan 1"
    pose_target = geometry_msgs.msg.Pose()
    
    #pose_target = PoseStamped()
    #pose_target.header.frame_id="/world"
    pose_target.orientation.w = 1.0
    pose_target.position.x = 0.8
    pose_target.position.y = -0.8
    pose_target.position.z = -0.15
    
    #pose_target=group.set
    #pose_target=group.set_random_target()
    group.set_planner_id("RRTConnectkConfigDefault")
    #group.set_start_state_to_current_state()
    #group.set_path_constraints(0)
    group.set_goal_orientation_tolerance(0.1)
    group.set_goal_position_tolerance(0.1)
    group.set_goal_joint_tolerance(0.1)
    group.set_goal_tolerance(0.1)
    group.set_planning_time(10)
    #group.set_random_target()
    group.set_pose_target(pose_target)


    plan1 = group.plan(pose_target)

    print "============ Waiting while RVIZ displays plan1..."
    #rospy.sleep(5)
    #Uncomment below line when working with a real robot
    group.go(wait=True)

    while not rospy.is_shutdown():
        rospy.loginfo("so e mist")
        rospy.sleep(1)
        rospy.spin()







if __name__ == '__main__':
        try:
                main()
        except rospy.ROSInterruptException:
                pass





"""
from moveit_python import *
import baxter_interface
from geometry_msgs.msg import PoseStamped
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from moveit_msgs.msg import MoveItErrorCodes





def get_real_coords(slot, qr_pose):

    d_x=5
    d_y=5
    n=7 
    gr= MoveGroupInterface("right_arm", "base")

    qr_x=1
    qr_y=2

    position = qr_pose.pose.position
    quat = qr_pose.pose.orientation
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
    goal_pose.header.frame_id = "/world"
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.pose.position.x = x_pose
    goal_pose.pose.position.y = y_pose
    goal_pose.pose.position.z = qr_pose.pose.position.z
    goal_pose.pose.orientation.x = qr_pose.pose.orientation.x
    goal_pose.pose.orientation.y = qr_pose.pose.orientation.y
    goal_pose.pose.orientation.z = qr_pose.pose.orientation.z
    goal_pose.pose.orientation.w = qr_pose.pose.orientation.w
    goal_pose=qr_pose
    gr.moveToPose(goal_pose, "right_gripper", plan_only=False)



def  main():
     

        rospy.init_node('grasping', anonymous=True)
        rate = rospy.Rate(10) # 10hz

        slot=15
        start_pose=PoseStamped()
        start_pose.header.frame_id="/world"
        start_pose.header.stamp = rospy.Time.now()
        start_pose.pose.position.x = 0.4
        start_pose.pose.position.y = -0.9
        start_pose.pose.position.z = 0.15
        start_pose.pose.orientation.x = 0
        start_pose.pose.orientation.y = 0
        start_pose.pose.orientation.z = 0
        start_pose.pose.orientation.w = 1.0
        #start_pose.pose = rotate_pose_msg_by_euler_angles(start_pose.pose, 0.0, 0.0, 0.0)
        #g = MoveGroupInterface("right_arm", "base")
        #pos1 = [-1.441426162661994, 0.8389151064712133, 0.14240920034028015, -0.14501001475655606, -1.7630090377446503, -1.5706376573674472, 0.09225918246029519,1.7238109084167481, 1.7169079948791506, 0.36930587426147465, -0.33249033539428713, -1.2160632682067871, 1.668587600115967, -1.810097327636719]
        #jts_right = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
        #g.moveToJointPosition(jts_right, pos1, plan_only=False)
        rospy.loginfo(" In start pos: ")
        p = PlanningSceneInterface("/world")
        p.clear()
        p.waitForSync()    

        gr= MoveGroupInterface("right_arm", "/world",plan_only=False)

        #rospy.loginfo(" current pose: " + gr.get_current_pose().pose)
        gr.setPlannerId("RRTConnectkConfigDefault")
        gr.moveToPose(start_pose, "right_gripper", plan_only=False,tolerance=0.1,planner_id="RRTConnectkConfigDefault")
        #gr.get_move_action().wait_for_result()
        result =gr.get_move_action().get_result()
        if result:
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Hello there!")
            else:
                rospy.logerr("Arm goal in state: %s",
                gr.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")
        #rospy.loginfo(" Result:  "  )
        #rospy.loginfo(" in greif pos ")




        #get_real_coords(slot,start_pose)


        rightgripper = baxter_interface.Gripper('right')
        #leftgripper.calibrate()
        rightgripper.open()
        #p = PlanningSceneInterface("base")
        #p.addCube("my_cube", 0.75, 1, 0, -0.5)  # size, x,y,z
        #p.addCube("my_cube", 0.01, 0.75, -0.4, -0.1)  # size, x,y,z
        #g = MoveGroupInterface("right_arm", "base")
        #pos1 = [-1.441426162661994, 0.8389151064712133, 0.14240920034028015, -0.14501001475655606, -1.7630090377446503, -1.5706376573674472, 0.09225918246029519,1.7238109084167481, 1.7169079948791506, 0.36930587426147465, -0.33249033539428713, -1.2160632682067871, 1.668587600115967, -1.810097327636719]
        #jts_right = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
        #g.moveToJointPosition(jts_right, pos1, plan_only=False)
 


        while not rospy.is_shutdown():
                rospy.loginfo("Fuesse")
                rate.sleep()
                #rightgripper.open()




if __name__ == '__main__':
        try:
                main()
        except rospy.ROSInterruptException:
                pass
"""

