#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import sqrt, pow, atan2
from collections import namedtuple
from tf import transformations

line_following = True
pub_cmd_vel = None
pub_point = None
bug_initialized = False
trackline = None
switch_point = None
current_pose = None
goal_reached = False

Line = namedtuple('Line', ['a', 'b', 'c'])

def run(self):
    rospy.spin()

def dist_from_trackline(point):
  return abs(trackline.a * point[0] + trackline.b * point[1] + trackline.c) / \
         sqrt(pow(trackline.a, 2) + pow(trackline.b, 2))


def dist_between_points(point1, point2):
  return sqrt(pow(point2[0] - point1[0], 2) + pow(point2[1] - point1[1], 2))

def odomCallback(odom):
  global trackline
  global switch_point
  global current_pose
  global bug_initialized
  global line_following
  global goal_reached

  euler = transformations.euler_from_quaternion((odom.pose.pose.orientation.x,
                                                 odom.pose.pose.orientation.y,
                                                 odom.pose.pose.orientation.z,
                                                 odom.pose.pose.orientation.w))

  current_pose = (odom.pose.pose.position.x,
                  odom.pose.pose.position.y,
                  euler[2])

  print(f'Distance to goal: {dist_between_points((current_pose[0], current_pose[1]), (2, 8))}')
  if dist_between_points((current_pose[0], current_pose[1]), (2, 8)) < 0.3:
    print(f'Goal reached!')
    goal_reached = True
    return

  # Initilaize bug algorithm by computing the line to goal
  if not bug_initialized:
    bug_initialized = True
    point2 = (2, 8) # TODO: param this
    trackline = Line(point2[1] - current_pose[1], # a
                     current_pose[0] - point2[0], # b
                     current_pose[1] * (point2[0] - current_pose[0]) - \
                     current_pose[0] * (point2[1] - current_pose[1])) # c
    switch_point = current_pose

  # Switch to back to line tracking
  if not line_following:
    print(f'Dist from trackline: {dist_from_trackline((current_pose[0], current_pose[1]))}')
  if not line_following and \
     dist_from_trackline((current_pose[0], current_pose[1])) < 0.05 and \
     dist_between_points((current_pose[0], current_pose[1]), switch_point) > 0.2:
    line_following = True


def scanCallback(scan):
    global pub_cmd_vel
    global line_following
    global current_pose
    global bug_initialized
    global goal_reached
    global switch_point

    if not bug_initialized:
      return

    obstacle_left = False
    obstacle_front = False
    obstacle_right = False
    cnt_left = 0
    cnt_front = 0
    cnt_right = 0
    range_left = 0
    range_front = 0
    range_right = 0

    obst_dist = 0.5

    front_angle = 30 / 180 * 3.14
    side_angle = 60 / 180 * 3.14

    obstacle_right = min(min(scan.ranges[int(-(scan.angle_min + side_angle) / scan.angle_increment):int(-(scan.angle_min + front_angle) / scan.angle_increment)]), 10) < obst_dist
    obstacle_front = min(min(scan.ranges[int(-(scan.angle_min + front_angle) / scan.angle_increment):int(-(scan.angle_min - front_angle) / scan.angle_increment)]), 10) < obst_dist
    obstacle_left = min(min(scan.ranges[int(-(scan.angle_min - front_angle) / scan.angle_increment):int(-(scan.angle_min - side_angle) / scan.angle_increment)]), 10) < obst_dist

    if line_following and obstacle_front:
        line_following = False
        switch_point = current_pose

    twist = Twist()
    twist.linear.x = 0.2

    if goal_reached:
      twist.linear.x = 0.0

    elif line_following:
      angle = atan2(trackline.a, -trackline.b) - current_pose[2]
      if abs(angle) < 1 / 180 * 3.14:
        twist.linear.x = 0.5
      elif angle > 0.0:
        twist.linear.x = 0.0
        twist.angular.z = 0.2
      else:
        twist.linear.x = 0.0
        twist.angular.z = -0.2

    else:
        twist.linear.x = 0.0
        twist.angular.z = 0.3
        if not obstacle_left and not obstacle_front and obstacle_right:
            twist.linear.x = 0.5
            twist.angular.z = 0.0
        elif not obstacle_left and not obstacle_front and not obstacle_right:
            twist.linear.x = 0.2
            twist.angular.z = -0.3
        elif obstacle_left and not obstacle_front and not obstacle_right:
            twist.linear.x = 0.2
            twist.angular.z = -0.3
        elif obstacle_left and not obstacle_front and obstacle_right:
            twist.linear.x = 0.2
            twist.angular.z = -0.3

    point = PointStamped()
    point.header.frame_id = 'odom'
    point.header.stamp = rospy.Time.now()
    point.point.x = 2
    point.point.y = 8
    pub_point.publish(point)
    pub_cmd_vel.publish(twist)


if __name__ == '__main__':
    rospy.init_node('bug')
    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    pub_point = rospy.Publisher('goal', PointStamped, queue_size=1)

    sub_odom = rospy.Subscriber('odom', Odometry, odomCallback, queue_size=1)
    sub_scan = rospy.Subscriber('scan', LaserScan, scanCallback, queue_size=1)
    rospy.spin()
