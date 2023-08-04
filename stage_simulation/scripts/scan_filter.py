#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from math import cos, sin

class ScanFilter:
  def __init__(self):
    self.pub_marker = rospy.Publisher("marker", Marker, queue_size=10)
    self.sub_laser_scan = rospy.Subscriber(
      "base_scan", LaserScan, self.laser_scan_callback)

    # definicija dijela Marker poruke koji se ne mjenja
    self.marker = Marker()
    self.marker.header.stamp = rospy.Time.now()
    self.marker.header.frame_id = "base_laser_link"
    self.marker.ns = "base_laser_link"
    self.marker.type = self.marker.LINE_STRIP
    self.marker.action = self.marker.MODIFY
    self.marker.pose.orientation.w = 1.0
    self.marker.scale.x = 0.1
    self.marker.scale.y = 0.1
    self.marker.scale.z = 0.1
    self.marker.color.a = 1.0
    self.marker.lifetime = rospy.Duration(1)

    # definicija boje markera
    self.color = ColorRGBA()
    self.color.r = 10
    self.color.g = 10
    self.color.b = 255
    self.color.a = 255

  def laser_scan_callback(self, scan):
    self.marker.points.clear()
    self.marker.colors.clear()
    # petlja za prolazak kroz sve tocke skena
    for k in range(0, round((scan.angle_max - scan.angle_min) / \
                            scan.angle_increment)):
      range_k = scan.ranges[k]
      # ako je range manji ili veci od dozvoljene udaljenosti
      # potrebno ga je odbaciti
      if range_k < scan.range_min or range_k > scan.range_max:
        continue
      # racunanje kuta laserske zrake
      angle = scan.angle_min + scan.angle_increment * k
      # provjera pada li zraka unutar filtra i dodavanje u marker
      if angle >= 0.5 and angle <= 1.0:
        point = Point32()
        point.x = range_k * cos(angle)
        point.y = range_k * sin(angle)
        self.marker.points.append(point)
        self.marker.colors.append(self.color)
    self.pub_marker.publish(self.marker)

if __name__ == '__main__':
  rospy.init_node('scan_filter')
  sf = ScanFilter()
  rospy.spin()

