#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pi
from tf import transformations
from bug_navigation.srv import SetBugGoal, SetBugGoalResponse

LINE_FOLLOWING = 0
OBSTACLE_FOLLOWING = 1

class BugNavigation:
    def __init__(self):
        self.following_stage = LINE_FOLLOWING
        self.trackline = None
        self.switch_point = None
        self.current_pose = None
        self.goal_reached = True
        self.goal = None

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_point = rospy.Publisher('goal', PointStamped, queue_size=1)

        self.sub_odom = rospy.Subscriber('odom', Odometry, self.odomCallback, queue_size=1)
        self.sub_scan = rospy.Subscriber('scan', LaserScan, self.scanCallback, queue_size=1)

        self.set_bug_goal_srv = rospy.Service('set_bug_goal', SetBugGoal, self.set_bug_goal)


    def set_bug_goal(self, req):
        # ukoliko je navigacija vec aktivna vrati False
        if not self.goal_reached:
            return SetBugGoalResponse(False)

        # resetiranje inicijalizacije bug algoritma
        self.goal_reached = False
        self.goal = (req.goal.x, req.goal.y)
        self.following_stage = LINE_FOLLOWING

        # objava tocke cilja, za vizualizaciju
        point = PointStamped()
        point.header.frame_id = 'odom'
        point.header.stamp = rospy.Time.now()
        point.point = req.goal
        self.pub_point.publish(point)

        # izracun pravca do cilja
        self.trackline = (self.goal[1] - self.current_pose[1], # a
                          self.current_pose[0] - self.goal[0], # b
                          self.current_pose[1] * (self.goal[0] - self.current_pose[0]) - \
                          self.current_pose[0] * (self.goal[1] - self.current_pose[1])) # c
        return SetBugGoalResponse(True)


    def dist_from_trackline(self, trackline, point):
        return abs(trackline[0] * point[0] + trackline[1] * point[1] + trackline[2]) / sqrt(trackline[0]**2 + trackline[1]**2)


    def dist_between_points(self, point1, point2):
        return sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)


    def odomCallback(self, odom):
        # dohvacanje trenutne poze
        euler = transformations.euler_from_quaternion((odom.pose.pose.orientation.x,
                                                       odom.pose.pose.orientation.y,
                                                       odom.pose.pose.orientation.z,
                                                       odom.pose.pose.orientation.w))
        self.current_pose = (odom.pose.pose.position.x, odom.pose.pose.position.y, euler[2])

        # ako je cilj dostignut nema potrebe provjeravati algoritam
        if self.goal_reached:
            return

        # provjera je li cilj dostignut
        if self.dist_between_points(self.current_pose, self.goal) < rospy.get_param('bug_tolerance', 0.3):
            self.goal_reached = True
            return

        # promijena rezima rada u slijedenje direktnog pravca
        if self.following_stage == OBSTACLE_FOLLOWING and \
           self.dist_from_trackline(self.trackline, self.current_pose) < 0.05 and \
           self.dist_between_points(self.current_pose, self.switch_point) > 0.2:
            self.following_stage = LINE_FOLLOWING


    def scanCallback(self, scan):
        twist = Twist()

        # ako je cilj dostignut vozilo treba zaustaviti
        if self.goal_reached:
            self.pub_cmd_vel.publish(twist)
            return

        # obrada skena i otkrivanje prepreke
        obst_dist = rospy.get_param('~obstacle_detection_distance', 0.5)
        front_angle = rospy.get_param('~front_angle', 30 / 180 * pi)
        side_angle = rospy.get_param('~side_angle', 60 / 180 * pi)

        obstacle_left = min(scan.ranges[int((front_angle - scan.angle_min) / scan.angle_increment):int((side_angle - scan.angle_min) / scan.angle_increment)]) < obst_dist
        obstacle_front = min(scan.ranges[int(-(front_angle + scan.angle_min) / scan.angle_increment):int((front_angle - scan.angle_min) / scan.angle_increment)]) < obst_dist
        obstacle_right = min(scan.ranges[int(-(side_angle + scan.angle_min) / scan.angle_increment):int(-(front_angle + scan.angle_min) / scan.angle_increment)]) < obst_dist


        # promijena rezima rada u slijedenje prepreke
        if self.following_stage == LINE_FOLLOWING and obstacle_front:
            self.following_stage = OBSTACLE_FOLLOWING
            self.switch_point = self.current_pose

        # racunanje brzine ovisno o rezimu rada
        if self.following_stage == LINE_FOLLOWING:
            twist = self.line_following()
        else:
            twist = self.obstacle_following(obstacle_left, obstacle_front, obstacle_right)

        self.pub_cmd_vel.publish(twist)


    def line_following(self):
        twist = Twist()
        angle = atan2(self.trackline[0], -self.trackline[1]) - self.current_pose[2]

        # ako je robot poravnat s pravcem treba krenuti prema cilju
        if abs(angle) < 1 / 180 * pi:
            twist.linear.x = rospy.get_param('~v_following', 0.5)

        # ako robot orijentacijom odstupa od orijentacije pravca potrebno je
        # poravnati robota, u lijevo ili u desno ovisno o razlici kutova
        elif angle > 0.0:
            twist.angular.z = abs(rospy.get_param('~w_in_place_rotation', 0.3))
        else:
            twist.angular.z = -abs(rospy.get_param('~w_in_place_rotation', 0.3))

        return twist


    def obstacle_following(self, obstacle_left, obstacle_front, obstacle_right):
        twist = Twist()

        # postavljene vrijednosti za poravnanje s preprekom
        twist.linear.x = 0.0
        twist.angular.z = rospy.get_param('~w_in_place_rotation', 0.3)

        # slijedenje prepreke
        if (not obstacle_left and not obstacle_front and obstacle_right) or \
            (obstacle_left and not obstacle_front and obstacle_right):
            twist.linear.x = rospy.get_param('~v_following', 0.5)
            twist.angular.z = 0.0

        # pronalazak konture prepreke
        elif (not obstacle_left and not obstacle_front and not obstacle_right) or \
             (obstacle_left and not obstacle_front and not obstacle_right):
            twist.linear.x = rospy.get_param('~v_turning', 0.2)
            twist.angular.z = rospy.get_param('~w_turning', -0.3)

        return twist


if __name__ == '__main__':
    rospy.init_node('bug_navigation')
    bn = BugNavigation()
    rospy.spin()
