#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

def odomCallback(data):
  v = data.twist.twist.linear.x
  w = data.twist.twist.angular.z
  d = rospy.get_param('d', 0.19)
  r = rospy.get_param('wheel_radius', 0.2)
  vl = v - w * d
  vr = 2 * v - vl
  publ.publish(vl / r)
  pubr.publish(vr / r)

if __name__ == '__main__':
  rospy.init_node('wheel_speed_publisher')

  publ = rospy.Publisher("left_wheel_speed", Float64, queue_size=10)
  pubr = rospy.Publisher("right_wheel_speed", Float64, queue_size=10)

  sub_odom = rospy.Subscriber("odom", Odometry, odomCallback)

  rospy.spin()

