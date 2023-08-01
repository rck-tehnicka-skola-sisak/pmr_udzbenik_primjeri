#!/usr/bin/env python3
import rospy
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
  rospy.init_node('map_to_odom')
  broadcaster = StaticTransformBroadcaster()
  static_transformStamped = TransformStamped()

  static_transformStamped.header.stamp = rospy.Time.now()
  static_transformStamped.header.frame_id = "map"
  static_transformStamped.child_frame_id = "odom"

  static_transformStamped.transform.rotation.w = 1
  broadcaster.sendTransform(static_transformStamped)
  rospy.spin()

