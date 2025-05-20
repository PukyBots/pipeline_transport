#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
import math

class CrossroadController:
    def __init__(self):
        rospy.init_node("crossroad_controller")

        self.platform_name = "cross_platform"
        self.pod_name = "transport_capsule"
        self.cross_position = (0.0, 0.0)
        self.last_rotation = 0.0  # Current rotation in radians

        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.monitor_loop()

    def is_pod_at_cross(self, pod_x, pod_y):
        return abs(pod_x - self.cross_position[0]) < 0.5 and abs(pod_y - self.cross_position[1]) < 0.5

    def rotate_platform(self, angle_rad):
        state = ModelState()
        state.model_name = self.platform_name
        state.pose.position.x = 0.0
        state.pose.position.y = 0.0
        state.pose.position.z = 0.75
        state.pose.orientation.z = math.sin(angle_rad / 2)
        state.pose.orientation.w = math.cos(angle_rad / 2)
        state.reference_frame = "world"
        try:
            self.set_state(state)
            rospy.loginfo(f"Rotated platform to {math.degrees(angle_rad)}°")
            self.last_rotation = angle_rad
        except:
            rospy.logerr("Failed to rotate platform")

    def monitor_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                pod_state = self.get_state(self.pod_name, "world")
                pod_x = pod_state.pose.position.x
                pod_y = pod_state.pose.position.y

                if self.is_pod_at_cross(pod_x, pod_y):
                    if abs(pod_y) > abs(pod_x):  # Going N/S
                        self.rotate_platform(math.pi / 2)  # 90°
                    else:
                        self.rotate_platform(0.0)  # 0°

            except:
                rospy.logwarn("Could not get model state")

            rate.sleep()

if __name__ == "__main__":
    try:
        CrossroadController()
    except rospy.ROSInterruptException:
        pass
