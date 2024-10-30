#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from filterpy.kalman import KalmanFilter

class KalmanFilterNode:
    def __init__(self):
        rospy.init_node('kalman_filter', anonymous=True)
        self.odom_sub = rospy.Subscriber("/noisy_odom", Odometry, self.filter_odom)
        self.filtered_odom_pub = rospy.Publisher("/filtered_odom", Odometry, queue_size=10)
        
        # Kalman filter initialization
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.x = np.array([0., 0., 0., 0.])  # initial state (x, y, vx, vy)
        self.kf.F = np.array([[1., 0., 1., 0.],
                              [0., 1., 0., 1.],
                              [0., 0., 1., 0.],
                              [0., 0., 0., 1.]])  # state transition matrix
        self.kf.H = np.array([[1., 0., 0., 0.],
                              [0., 1., 0., 0.]])  # measurement matrix
        self.kf.P *= 1000.  # covariance matrix
        self.kf.R = np.array([[0.1, 0.],
                              [0., 0.1]])  # measurement noise
        self.kf.Q = np.eye(4) * 0.01  # process noise

    def filter_odom(self, msg):
        z = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.kf.predict()
        self.kf.update(z)
        filtered_msg = msg
        filtered_msg.pose.pose.position.x = self.kf.x[0]
        filtered_msg.pose.pose.position.y = self.kf.x[1]
        filtered_msg.twist.twist.linear.x = self.kf.x[2]
        self.filtered_odom_pub.publish(filtered_msg)

if __name__ == '__main__':
    try:
        kf_node = KalmanFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
