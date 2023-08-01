#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class RobotVeloictyPublisher:
  # konstruktor
  def __init__(self):
    # definiranje clanova klase
    self.pub_twist = rospy.Publisher("robot_vel", Twist, queue_size=10)
    self.sub_left_wheel = rospy.Subscriber(
      "left_wheel_speed", Float64, self.leftWheelSpeedCallback)
    self.sub_right_wheel = rospy.Subscriber(
      "right_wheel_speed", Float64, self.rightWheelSpeedCallback)
    self.wl = 0.0 # clan za spremanje kutne brzine proizvoljnog kotaca

  def leftWheelSpeedCallback(self, data):
    self.wl = data.data

  # Racunanje brzine robota i objavljivanje na robot_vel temu
  def rightWheelSpeedCallback(self, data):
    r = rospy.get_param('wheel_radius', 0.2) # radijus kotaca
    d = rospy.get_param(
      'd', 0.19) # udaljenost od kotaca do baze robota
    vr = data.data * r # linearna brzina desnog kotaca
    vl = self.wl * r # linearna brzina lijevog kotaca
    twist = Twist()
    twist.linear.x = (vr + vl) / 2 # linearna brzina robota
    twist.angular.z = (vr - vl) / (2 * d) # kutna brzina robota
    self.pub_twist.publish(twist)

if __name__ == '__main__':
  rospy.init_node('robot_velocity_publisher')
  rvp = RobotVeloictyPublisher()
  rospy.spin()
