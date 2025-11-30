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
from tf import transformations as tf_math
import copy

# --- HỆ SỐ TỶ LỆ ---
SCALE_FACTOR = rospy.get_param("~robot_scale_factor", 0.3) 
# -------------------

class DataHolder:
    def __init__(self):
        self.pose_vr = PoseStamped()
        
        # Biến xử lý nút bấm
        self.trigger_pressed_prev = False # Trạng thái cò ở frame trước
        self.toggle_state = False         # Trạng thái Bật/Tắt (True = Đang điều khiển)
        self.just_toggled_on = False      # Cờ báo hiệu vừa mới Bật
        
        self.sub_pose = rospy.Subscriber("/vive_rightPose", PoseStamped, self.pose_vrCallback)
        self.sub_trigger = rospy.Subscriber("/vive_rightTrigger", Float32, self.triggerCallback)

    def pose_vrCallback(self, msg):
        self.pose_vr = msg
        
    def triggerCallback(self, msg):
        # Chuyển đổi tín hiệu analog (0.0-1.0) sang bool
        is_pressed_now = msg.data > 0.99
        
        # Phát hiện sườn dương (Rising Edge): Vừa mới bấm xuống
        if is_pressed_now and not self.trigger_pressed_prev:
            # Đảo trạng thái (Toggle)
            self.toggle_state = not self.toggle_state
            
            if self.toggle_state:
                self.just_toggled_on = True # Đánh dấu là vừa mới bật
                rospy.loginfo(">>> ĐÃ BẬT ĐIỀU KHIỂN (ON) <<<")
            else:
                rospy.loginfo(">>> ĐÃ TẮT ĐIỀU KHIỂN (OFF) <<<")
        
        # Lưu trạng thái để so sánh lần sau
        self.trigger_pressed_prev = is_pressed_now

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
    rospy.init_node('vive_control_toggle_mode', anonymous=True)

    # --- 1. CẦU NỐI TF (Giữ nguyên) ---
    broadcaster = StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "vive_world"
    t.child_frame_id = "base_link"
    
    # Calibration (Điền số thực tế của bạn vào đây)
    t.transform.translation.x = 1.0 
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.8
    
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
    
    target_pose_pub = rospy.Publisher("/vive_target_pose", PoseStamped, queue_size=1)

    go_to_initial_state(move_group)
    rate = rospy.Rate(10) 
    
    start_robot_pose = PoseStamped()
    start_vive_pose = PoseStamped()
    
    rospy.loginfo("Sẵn sàng. Bấm cò 1 lần để BẬT. Bấm lần nữa để TẮT.")

    while not rospy.is_shutdown():
        # Luôn cập nhật TF
        pose_in_vive_frame = data.pose_vr
        if not pose_in_vive_frame.header.frame_id:
            pose_in_vive_frame.header.frame_id = "vive_world"

        try:
            current_vive_tf_pose = tf_buffer.transform(
                pose_in_vive_frame,
                robot_base_frame,
                rospy.Duration(1.0)
            )

            # --- KIỂM TRA TRẠNG THÁI TOGGLE ---
            if data.toggle_state: 
                # (Robot đang ở chế độ HOẠT ĐỘNG)

                # --- 1. KHOẢNH KHẮC VỪA BẬT (Reset gốc) ---
                if data.just_toggled_on:
                    # Lấy pose hiện tại làm mốc 0
                    start_robot_pose = move_group.get_current_pose(move_group.get_end_effector_link())
                    start_vive_pose = copy.deepcopy(current_vive_tf_pose)
                    data.just_toggled_on = False # Xóa cờ
                
                # --- 2. TÍNH TOÁN LIÊN TỤC (Relative Control) ---
                
                # Tính Delta Position (Có Scale)
                delta_x = (current_vive_tf_pose.pose.position.x - start_vive_pose.pose.position.x) * SCALE_FACTOR
                delta_y = (current_vive_tf_pose.pose.position.y - start_vive_pose.pose.position.y) * SCALE_FACTOR
                delta_z = (current_vive_tf_pose.pose.position.z - start_vive_pose.pose.position.z) * SCALE_FACTOR

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = robot_base_frame
                goal_pose.pose.position.x = start_robot_pose.pose.position.x + delta_x
                goal_pose.pose.position.y = start_robot_pose.pose.position.y + delta_y
                goal_pose.pose.position.z = start_robot_pose.pose.position.z + delta_z

                # Tính Delta Orientation
                q_robot_start = [start_robot_pose.pose.orientation.x, start_robot_pose.pose.orientation.y, start_robot_pose.pose.orientation.z, start_robot_pose.pose.orientation.w]
                q_vive_start = [start_vive_pose.pose.orientation.x, start_vive_pose.pose.orientation.y, start_vive_pose.pose.orientation.z, start_vive_pose.pose.orientation.w]
                q_vive_current = [current_vive_tf_pose.pose.orientation.x, current_vive_tf_pose.pose.orientation.y, current_vive_tf_pose.pose.orientation.z, current_vive_tf_pose.pose.orientation.w]

                q_vive_delta = tf_math.quaternion_multiply(
                    tf_math.quaternion_inverse(q_vive_start), 
                    q_vive_current
                )
                q_robot_new = tf_math.quaternion_multiply(q_robot_start, q_vive_delta)

                goal_pose.pose.orientation.x = q_robot_new[0]
                goal_pose.pose.orientation.y = q_robot_new[1]
                goal_pose.pose.orientation.z = q_robot_new[2]
                goal_pose.pose.orientation.w = q_robot_new[3]

                # --- 3. GỬI LỆNH ---
                target_pose_pub.publish(goal_pose)
                move_group.set_pose_target(goal_pose)
                move_group.go(wait=True)
            
            else:
                # (Robot đang ở chế độ TẮT/NGHỈ)
                # Đảm bảo robot dừng lại và không có mục tiêu tồn đọng
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
