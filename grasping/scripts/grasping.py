#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from moveit_python import *
import baxter_interface

def  main():
   

    rospy.init_node('grasping', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    rightgripper = baxter_interface.Gripper('right')
    #leftgripper.calibrate()
    rightgripper.open()

    g = MoveGroupInterface("right_arm", "base")
    pos1 = [-1.441426162661994, 0.8389151064712133, 0.14240920034028015, -0.14501001475655606, -1.7630090377446503, -1.5706376573674472, 0.09225918246029519,1.7238109084167481, 1.7169079948791506, 0.36930587426147465, -0.33249033539428713, -1.2160632682067871, 1.668587600115967, -1.810097327636719]
    jts_right = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    g.moveToJointPosition(jts_right, pos1, plan_only=False)
    rightgripper.close()


    while not rospy.is_shutdown():
        rospy.loginfo("Fuesse")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
