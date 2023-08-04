#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from math import cos, sin

class OdometryPublisher:
  def __init__(self):
    self.pub_odom = rospy.Publisher("odom_check", Odometry, queue_size=10)
    self.sub_twist = rospy.Subscriber("robot_vel", Twist, self.twist_callback)

    # postavljanje osnovnih vrijednosi odometry poruke
    self.odom = Odometry()
    self.odom.header.frame_id = "self.odom"
    self.odom.child_frame_id = "base_link"
    # Postavljanje pocetne orijentacije u 0
    quat_init = transformations.quaternion_from_euler(0, 0, 0)
    self.odom.pose.pose.orientation.x = quat_init[0]
    self.odom.pose.pose.orientation.y = quat_init[1]
    self.odom.pose.pose.orientation.z = quat_init[2]
    self.odom.pose.pose.orientation.w = quat_init[3]
    # prev_time se koristi za racuanje delta t
    self.prev_time = rospy.Time.now()

  # izracun odometrije
  def twist_callback(self, data):
    dt = (rospy.Time.now() - self.prev_time).to_sec()
    self.prev_time = rospy.Time.now()

    v = data.linear.x
    w = data.angular.z

    dtheta = w * dt

    # racuanje trenutne orijentacije iz kvaterniona
    quat = self.odom.pose.pose.orientation
    (r, p, theta) = transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))

    # racuanje nove poze
    theta_new = theta + dtheta
    # mora se paziti na posebni slucaj kada je kutna brzina 0
    if abs(w) > 1.e-3:
      self.odom.pose.pose.position.x += v / w * (sin(theta_new) - sin(theta))
      self.odom.pose.pose.position.y -= v / w * (cos(theta_new) - cos(theta))
    else:
      dl = v * dt
      self.odom.pose.pose.position.x += dl * cos(theta_new)
      self.odom.pose.pose.position.y += dl * sin(theta_new)

    # vracanje nove orijentacije u kvaternion
    quat_new = transformations.quaternion_from_euler(0, 0, theta_new)
    self.odom.pose.pose.orientation.x = quat_new[0]
    self.odom.pose.pose.orientation.y = quat_new[1]
    self.odom.pose.pose.orientation.z = quat_new[2]
    self.odom.pose.pose.orientation.w = quat_new[3]

    self.odom.twist.twist.linear.x = v
    self.odom.twist.twist.angular.z = w

    self.odom.header.stamp = rospy.Time.now()
    self.pub_odom.publish(self.odom)

if __name__ == '__main__':
  rospy.init_node('odometry_publisher')
  op = OdometryPublisher()
  rospy.spin()
