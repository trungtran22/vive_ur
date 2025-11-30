#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, TransformStamped
import moveit_commander
from math import tau, pi
import tf2_ros
import tf2_geometry_msgs
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
# Import thư viện toán học Quaternion
from tf import transformations as tf_math
import copy

# --- HỆ SỐ TỶ LỆ ---
SCALE_FACTOR = rospy.get_param("~robot_scale_factor", 0.3) 
# -------------------

class DataHolder:
    def __init__(self):
        self.pose_vr = PoseStamped()
        self.trigger_value = 0.0
        self.is_pressed = False
        self.just_pressed = False
        
        self.sub_pose = rospy.Subscriber("/vive_rightPose", PoseStamped, self.pose_vrCallback)
        self.sub_trigger = rospy.Subscriber("/vive_rightTrigger", Float32, self.triggerCallback)

    def pose_vrCallback(self, msg):
        self.pose_vr = msg
        
    def triggerCallback(self, msg):
        new_val = msg.data
        new_state = new_val > 0.99
        if new_state and not self.is_pressed:
            self.just_pressed = True
        self.trigger_value = new_val
        self.is_pressed = new_state

def go_to_initial_state(move_group):
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -tau / 4
    joint_goal[2] = tau / 4
    joint_goal[3] = -tau / 4
    joint_goal[4] = -tau / 4
    joint_goal[5] = -tau/6
    move_group.go(joint_goal, wait=True)
    move_group.stop()

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('vive_control_full_relative', anonymous=True)

    # --- 1. CẦU NỐI TF (Giữ nguyên) ---
    broadcaster = StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "vive_world"
    t.child_frame_id = "base_link"
    
    # Calibration (Bạn tự điền số thực tế vào đây)
    t.transform.translation.x = 1.0 
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.8
    
    # Xoay trục Y-Up sang Z-Up
    q = tf_math.quaternion_from_euler(-pi/2, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    broadcaster.sendTransform(t)
    
    # --- 2. SETUP ---
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    data = DataHolder()
    robot_base_frame = move_group.get_planning_frame()
    
    # Publisher để debug mục tiêu trong RViz
    target_pose_pub = rospy.Publisher("/vive_target_pose", PoseStamped, queue_size=1)

    go_to_initial_state(move_group)
    rate = rospy.Rate(10) 
    
    # --- Biến lưu trạng thái bắt đầu ---
    start_robot_pose = PoseStamped()
    start_vive_pose = PoseStamped()
    
    rospy.loginfo("Sẵn sàng. Bóp cò để điều khiển (Vị trí + Xoay).")

    while not rospy.is_shutdown():
        # Lấy pose hiện tại của VIVE (đã transform sang frame robot)
        pose_in_vive_frame = data.pose_vr
        if not pose_in_vive_frame.header.frame_id:
            pose_in_vive_frame.header.frame_id = "vive_world"

        try:
            current_vive_tf_pose = tf_buffer.transform(
                pose_in_vive_frame,
                robot_base_frame,
                rospy.Duration(1.0)
            )

            if data.is_pressed:
                # --- KHI VỪA BÓP CÒ (Lưu mốc ban đầu) ---
                if data.just_pressed:
                    rospy.loginfo("--- KHÓA VỊ TRÍ & GÓC ---")
                    # Lấy pose hiện tại của robot thật
                    start_robot_pose = move_group.get_current_pose(move_group.get_end_effector_link())
                    # Lấy pose hiện tại của tay cầm VIVE
                    start_vive_pose = copy.deepcopy(current_vive_tf_pose)
                    data.just_pressed = False
                
                # --- KHI ĐANG GIỮ CÒ (Tính toán tương đối) ---
                
                # 1. TÍNH VỊ TRÍ (POSITION) - Có Scale
                delta_x = (current_vive_tf_pose.pose.position.x - start_vive_pose.pose.position.x) * SCALE_FACTOR
                delta_y = (current_vive_tf_pose.pose.position.y - start_vive_pose.pose.position.y) * SCALE_FACTOR
                delta_z = (current_vive_tf_pose.pose.position.z - start_vive_pose.pose.position.z) * SCALE_FACTOR

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = robot_base_frame
                goal_pose.pose.position.x = start_robot_pose.pose.position.x + delta_x
                goal_pose.pose.position.y = start_robot_pose.pose.position.y + delta_y
                goal_pose.pose.position.z = start_robot_pose.pose.position.z + delta_z

                # 2. TÍNH GÓC XOAY (ORIENTATION) - Relative Rotation
                # Công thức: Q_new = Q_robot_start * (Q_vive_start^-1 * Q_vive_current)
                
                # Lấy Quaternion dưới dạng list [x, y, z, w]
                q_robot_start = [start_robot_pose.pose.orientation.x, start_robot_pose.pose.orientation.y, start_robot_pose.pose.orientation.z, start_robot_pose.pose.orientation.w]
                q_vive_start = [start_vive_pose.pose.orientation.x, start_vive_pose.pose.orientation.y, start_vive_pose.pose.orientation.z, start_vive_pose.pose.orientation.w]
                q_vive_current = [current_vive_tf_pose.pose.orientation.x, current_vive_tf_pose.pose.orientation.y, current_vive_tf_pose.pose.orientation.z, current_vive_tf_pose.pose.orientation.w]

                # Tính Delta Rotation (Sự thay đổi góc của tay cầm)
                # Delta = Inverse(Start) * Current
                q_vive_delta = tf_math.quaternion_multiply(
                    tf_math.quaternion_inverse(q_vive_start), 
                    q_vive_current
                )

                # Áp dụng Delta đó vào Robot
                # New = Robot_Start * Delta
                q_robot_new = tf_math.quaternion_multiply(q_robot_start, q_vive_delta)

                # Gán vào Goal Pose
                goal_pose.pose.orientation.x = q_robot_new[0]
                goal_pose.pose.orientation.y = q_robot_new[1]
                goal_pose.pose.orientation.z = q_robot_new[2]
                goal_pose.pose.orientation.w = q_robot_new[3]

                # --- GỬI LỆNH ---
                target_pose_pub.publish(goal_pose) # Vẽ lên RViz
                move_group.set_pose_target(goal_pose)
                move_group.go(wait=True)
            
            else:
                if not data.just_pressed:
                    data.just_pressed = False
                    move_group.stop()
                    move_group.clear_pose_targets()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, f"Lỗi TF: {e}")
        
        rate.sleep()

    moveit_commander.roscpp_shutdown()

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
