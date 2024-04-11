#! /usr/bin/env python
import math
import rospy
import numpy as np
from math import pi
from geometry_msgs.msg import Twist, Pose, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class Env():
    def __init__(self, action_dim=2):
        self.goal_x = 1.0
        self.goal_y = 1.0
        self.heading = 0
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.pub_goal = rospy.Publisher('goal_position', PointStamped, queue_size=1)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.action_dim = action_dim
        self.resolution = 36
        self.angle = 0.
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        rospy.loginfo("Stopping TurtleBot")
        self.pub_cmd_vel.publish(Twist())
        rospy.sleep(1)

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        self.past_distance = goal_distance
        return goal_distance

    def getOdometry(self, odom):
        self.past_position = copy.deepcopy(self.position)
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        # print 'heading', heading
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 3)

    def getState(self, scan, past_action):
        scan_range = []
        heading = self.heading
        min_range = 0.16
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf') or scan.ranges[i] == float('inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]) or scan.ranges[i] == float('nan'):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        if min_range > min(scan_range) > 0:
            done = True

        obstacle_angle = np.argmin(scan_range) * self.resolution
        if obstacle_angle > pi:
            obstacle_angle -= 2 * pi

        elif obstacle_angle < -pi:
            obstacle_angle += 2 * pi

        obstacle_angle = round(obstacle_angle, 3)
        self.angle = obstacle_angle

        for pa in past_action:
            scan_range.append(pa)

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        # current_distance = self.getGoalDistace()
        if current_distance < 0.2:
            self.get_goalbox = True

        return scan_range + [heading, current_distance], done

    def setReward(self, state, done):
        goal = False
        current_distance = state[-1]
        heading = state[-2]

        distance_rate = (self.past_distance - current_distance)
        if distance_rate > 0:
            reward = 2.

        if distance_rate <= 0:
            reward = -2.

        # angle_reward = math.pi / 2 - abs(heading)
        # print('d', 500*distance_rate)
        # reward = round(10. * distance_rate + angle_reward, 2)
        self.past_distance = current_distance

        a, b, c, d = float('{0:.3f}'.format(self.position.x)), float('{0:.3f}'.format(self.past_position.x)), float(
            '{0:.3f}'.format(self.position.y)), float('{0:.3f}'.format(self.past_position.y))
        if a == b and c == d:
            # rospy.loginfo('\n<<<<<Stopped>>>>>\n')
            # print('\n' + str(a) + ' ' + str(b) + ' ' + str(c) + ' ' + str(d) + '\n')
            self.stopped += 1
            if self.stopped == 20:
                rospy.loginfo('Robot is in the same 20 times in a row')
                self.stopped = 0
                done = True
        else:
            # rospy.loginfo('\n>>>>> not stopped>>>>>\n')
            self.stopped = 0

        if done:
            rospy.loginfo("Collision!!")
            # reward = -500.
            reward = -100.
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            goal = True
            rospy.loginfo("Goal!!")
            # reward = 500.
            reward = 100.
            self.pub_cmd_vel.publish(Twist())

            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False

        return reward, done, goal

    def step(self, action, past_action):
        linear_vel = action[0]
        ang_vel = action[1]

        vel_cmd = Twist()
        vel_cmd.linear.x = linear_vel
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, done = self.getState(data, past_action)
        reward, done, goal = self.setReward(state, done)

        return np.asarray(state), reward, done, goal

    def reset(self):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass
        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False
        else:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
        self.goal_distance = self.getGoalDistace()
        self.publish_goal_position()
        state, _ = self.getState(data, [0]*self.action_dim)
        return np.asarray(state)

    def publish_goal_position(self):
        goal_position = PointStamped()
        goal_position.header.stamp = rospy.Time.now()
        goal_position.header.frame_id = 'map'  # Change this to the appropriate frame_id if needed
        goal_position.point.x = self.goal_x
        goal_position.point.y = self.goal_y
        goal_position.point.z = 0
        self.pub_goal.publish(goal_position)
