#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import random
import math

class ImuPublisher:
    def __init__(self):
        rospy.init_node('imu_publisher', anonymous=True)
        self.imu_pub = rospy.Publisher("/imu_data", Imu, queue_size=10)
        self.rate = rospy.Rate(10)

    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"
        # Adding some noise to IMU data
        imu_msg.angular_velocity.z = 0.1 + random.gauss(0, 0.01)
        imu_msg.linear_acceleration.x = 0.2 + random.gauss(0, 0.01)
        self.imu_pub.publish(imu_msg)

if __name__ == '__main__':
    try:
        imu_publisher = ImuPublisher()
        while not rospy.is_shutdown():
            imu_publisher.publish_imu()
            imu_publisher.rate.sleep()
    except rospy.ROSInterruptException:
        pass
