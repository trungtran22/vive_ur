#!/usr/bin/env python3
import sys
import copy
import rospy
from std_msgs.msg import String,Int32, Float32
from moveit_commander.conversions import pose_to_list
# from move_group_joint_state import MoveGroupPythonJointState
from geometry_msgs.msg import Pose,PoseStamped

import moveit_commander
from math import pi, tau

class DataHolder:
    def __init__(self):
        

            self.pose = Pose()
            self.pose_vr = PoseStamped()
            self.trigger = Float32()
            

            self.sub_pose = rospy.Subscriber("/vive_rightPose", PoseStamped, self.pose_vrCallback)
            self.sub_trigger = rospy.Subscriber("/vive_rightTrigger", Float32, self.triggerCallback)

    #vive publisher
    def pose_vrCallback(self,msg):
        self.pose_vr = msg
    def triggerCallback(self,msg):
        self.trigger = msg
    def make_pose(self,x,y,z):
        
        pose = Pose()
        pose.position.x = x -1.1
        pose.position.y = - (y + 0.28)
        pose.position.z = z + 1.0

        pose.orientation.x = 0
        pose.orientation.y = 1.0
        pose.orientation.z = 0
        pose.orientation.w = 0

        return pose
    
def go_to_initial_state():
        
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 4
        joint_goal[2] = tau / 4
        joint_goal[3] = -tau / 4
        joint_goal[4] = -tau / 4
        joint_goal[5] = -tau/6  # 1/6 of a turn
        move_group.go(joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()    
def vive_controlling():
    # move_group = moveit_commander.MoveGroupCommander("manipulator")
    data = DataHolder()
    while True:
        vr = data.pose_vr
        x_ = vr.pose.position.x
        y_ = vr.pose.position.y
        z_ = vr.pose.position.z
        print(x_)
        pose_goal = data.make_pose(x_,y_,z_)
        print(pose_goal)
        
    
        if data.trigger.data > 0.99:
            move_group.set_pose_target(pose_goal)
            move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()
        
if __name__=="__main__":
    #Init MOVEVIT
    rospy.init_node('vive_control', anonymous=True)
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    go_to_initial_state()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        vive_controlling()
        rate.sleep()
