# #!/usr/bin/env python3
# import rospy
# from std_msgs.msg import Float64
# import time

# def speed_commander():
#     pub = rospy.Publisher('/capsule/target_speed', Float64, queue_size=10)
#     rospy.init_node('speed_commander', anonymous=True)
#     rate = rospy.Rate(0.2)  # 0.2 Hz
    
#     speeds = [0.0]  # Test speed sequence
    
#     for speed in speeds:
#         if not rospy.is_shutdown():
#             rospy.loginfo(f"Setting target speed to: {speed} m/s")
#             pub.publish(speed)
#             time.sleep(5)  # Wait 5 seconds at each speed

# if __name__ == '__main__':
#     try:
#         speed_commander()
#     except rospy.ROSInterruptException:
#         pass