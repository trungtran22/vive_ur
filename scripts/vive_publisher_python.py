#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Float32, Int32, Float32MultiArray
from visualization_msgs.msg import Marker
from openvr import *
import math
from ctypes import sizeof # struct for GetControllerState

class VR_ControlHandler:
    
    def __init__(self):
        # data structure for 2 controllers (Left: Index 0, Right: Index 1)
        self.pController = [
            {'status': False, 'trigger': 0.0, 'touchpad': [0.0, 0.0], 'menu': 0, 'grip': 0, 'pose': PoseStamped()},
            {'status': False, 'trigger': 0.0, 'touchpad': [0.0, 0.0], 'menu': 0, 'grip': 0, 'pose': PoseStamped()}
        ]

class VivePublishing:
    def __init__(self):
        rospy.init_node('vive_publisher', anonymous=True)
        
        # ROS Publishers
        self.pub_controller_pose = [rospy.Publisher("/vive/controller/left/pose", PoseStamped, queue_size=1),
                                    rospy.Publisher("/vive/controller/right/pose", PoseStamped, queue_size=1)]
        self.pub_controller_trigger = [rospy.Publisher("/vive/controller/left/trigger", Float32, queue_size=1),
                                        rospy.Publisher("/vive/controller/right/trigger", Float32, queue_size=1)]
        self.pub_controller_menu = [rospy.Publisher("/vive/controller/left/buttons/menu", Int32, queue_size=1),
                                    rospy.Publisher("/vive/controller/right/buttons/menu", Int32, queue_size=1)]
        self.pub_controller_grip = [rospy.Publisher("/vive/controller/left/buttons/grip", Int32, queue_size=1),
                                    rospy.Publisher("/vive/controller/right/buttons/grip", Int32, queue_size=1)]
        self.pub_controller_touchpad = [rospy.Publisher("/vive/controller/left/touchpad", Float32MultiArray, queue_size=1),
                                        rospy.Publisher("/vive/controller/right/touchpad", Float32MultiArray, queue_size=1)]
        
        self.controlHandler = VR_ControlHandler()
        self.vr_pointer = None 
        
        # Init OpenVR
        try:
           
            self.vr_pointer = init(VRApplication_Scene) 
            if not self.vr_pointer:
                 rospy.logerr("OpenVR init returned None.")
        except Exception as e:
            rospy.logerr(f"OpenVR initialization failed: {e}")
            self.vr_pointer = None

    
    def remap_position(self, pose):
        
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        
        pose.position.x = -z
        pose.position.y = -x
        pose.position.z = y
        return pose

   
    def make_pose(self, controller_position, controller_orientation):
        
        controller_msg = PoseStamped()
        controller_msg.header.frame_id = "base" 

        # 1. Set Position
        controller_msg.pose.position.x = controller_position.v[0]
        controller_msg.pose.position.y = controller_position.v[1]
        controller_msg.pose.position.z = controller_position.v[2]
        
        # 2. Apply Position Remapping
        controller_msg.pose = self.remap_position(controller_msg.pose)
        
        # 3. Apply Orientation Remapping 
        # controller_msg.pose.orientation.w = controller_orientation.w;
        # controller_msg.pose.orientation.x = -controller_orientation.z;
        # controller_msg.pose.orientation.y = -controller_orientation.x;
        # controller_msg.pose.orientation.z = controller_orientation.y;
        controller_msg.pose.orientation.w = controller_orientation.w
        controller_msg.pose.orientation.x = -controller_orientation.z
        controller_msg.pose.orientation.y = -controller_orientation.x
        controller_msg.pose.orientation.z = controller_orientation.y

        return controller_msg

    
   
    def get_rotation(self, matrix):
       
        q = HmdQuaternion_t()
        q.w = math.sqrt(max(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2
        q.x = math.sqrt(max(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2
        q.y = math.sqrt(max(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2
        q.z = math.sqrt(max(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2
        q.x = math.copysign(q.x, matrix.m[2][1] - matrix.m[1][2])
        q.y = math.copysign(q.y, matrix.m[0][2] - matrix.m[2][0])
        q.z = math.copysign(q.z, matrix.m[1][0] - matrix.m[0][1])
        return q

    
    def get_position(self, matrix):
        
        vector = HmdVector3_t()
        vector.v[0] = matrix.m[0][3]
        vector.v[1] = matrix.m[1][3]
        vector.v[2] = matrix.m[2][3]
        return vector

    
    def get_controller_state(self):
        if not self.vr_pointer:
            return

        for deviceId in range(k_unMaxTrackedDeviceCount):
            cl = self.vr_pointer.GetTrackedDeviceClass(deviceId)
            
            if not self.vr_pointer.IsTrackedDeviceConnected(deviceId):
                continue
            
            if cl == TrackedDeviceClass_Controller:
                trackedDevicePose = TrackedDevicePose_t()
                controllerState = VRControllerState_t()
                
                
                self.vr_pointer.GetControllerStateWithPose(TrackingUniverseStanding, deviceId, controllerState, trackedDevicePose)
                
                role = self.vr_pointer.GetControllerRoleForTrackedDeviceIndex(deviceId)
                
               
                self.vr_pointer.GetControllerState(deviceId, controllerState, sizeof(controllerState))
                
               
                controllerIndex = 0 if role == TrackedControllerRole_LeftHand else 1
                
                # 
                self.controlHandler.pController[controllerIndex]['trigger'] = controllerState.rAxis[1].x # Axis 1 is Trigger
                self.controlHandler.pController[controllerIndex]['touchpad'] = [controllerState.rAxis[0].x, controllerState.rAxis[0].y] # Axis 0 is Touchpad
                self.controlHandler.pController[controllerIndex]['status'] = trackedDevicePose.bPoseIsValid
                
                
                menu_pressed = 1 if controllerState.ulButtonPressed & k_EButton_ApplicationMenu else 0
                grip_pressed = 1 if controllerState.ulButtonPressed & k_EButton_Grip else 0

                self.controlHandler.pController[controllerIndex]['menu'] = menu_pressed
                self.controlHandler.pController[controllerIndex]['grip'] = grip_pressed
                
                if trackedDevicePose.bPoseIsValid:
                    controller_position = self.get_position(trackedDevicePose.mDeviceToAbsoluteTracking)
                    controller_orientation = self.get_rotation(trackedDevicePose.mDeviceToAbsoluteTracking)
                    
                    # Pose message with logic remap position & orientation
                    controller_msg = self.make_pose(controller_position, controller_orientation)
                    self.controlHandler.pController[controllerIndex]['pose'] = controller_msg

    
    def publish_controller_state(self):
        for i in range(2):
            if self.controlHandler.pController[i]['status']:
                
                # 1. Publish Pose
                self.pub_controller_pose[i].publish(self.controlHandler.pController[i]['pose'])
                
                # 2. Publish Axis/Button Data (cần Float32/Int32 wrapper)
                self.pub_controller_trigger[i].publish(Float32(self.controlHandler.pController[i]['trigger']))
                self.pub_controller_menu[i].publish(Int32(self.controlHandler.pController[i]['menu']))
                self.pub_controller_grip[i].publish(Int32(self.controlHandler.pController[i]['grip']))
                
                # 3. Publish Touchpad Array
                touchpad_msg = Float32MultiArray(data=self.controlHandler.pController[i]['touchpad'])
                self.pub_controller_touchpad[i].publish(touchpad_msg)

    
    def run(self):
        if not self.vr_pointer:
            rospy.logerr("Unable to init VR runtime or initialization failed. Shutting down node.")
            return
            
        rospy.loginfo("Started vive_pose node and VR runtime initialized.")
        rate = rospy.Rate(10) 
        
        while not rospy.is_shutdown():
            self.get_controller_state()
            self.publish_controller_state()
            
            rate.sleep()
        
        self.shutdown()

    
    def shutdown(self):
        if self.vr_pointer:
            openvr.shutdown()
        rospy.loginfo("Vive publisher node shut down.")

if __name__ == '__main__':
    try:
        vive_pose_node = VivePublishing()
        vive_pose_node.run()
    except rospy.ROSInterruptException:
        pass
