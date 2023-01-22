#!/usr/bin/env python

# -----------------------------------------------------------------------------------------
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# -----------------------------------------------------------------------------------------


class Robot():
    def __init__(self):
        self.msg = None
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber(
            '/kobuki/laser/scan', LaserScan, self.callback)
        self.move = Twist()
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
    def callback(self, msg):
        self.msg = msg

        if self.should_proceed():
            self.move_forward()
        else:
            self.stop()
            self.turn(self.choose_direction())
        if self.should_proceed():
            self.stop_turning()
            self.move_forward()
        print(self.msg.ranges[719],
              self.msg.ranges[359], self.msg.ranges[0])
        self.pub.publish(self.move)

    def move_forward(self):
        self.linear_velocity = 0.25
        self.move.linear.x = self.linear_velocity

    def stop(self):
        self.linear_velocity = 0.0
        self.move.linear.x = self.linear_velocity

    def turn_left(self):
        self.angular_velocity = 3
        self.move.angular.z = self.angular_velocity

    def turn_right(self):
        self.angular_velocity = -3
        self.move.angular.z = self.angular_velocity

    def stop_turning(self):
        self.angular_velocity = 0.0
        self.move.angular.z = self.angular_velocity

    def should_proceed(self):
        if self.msg.ranges[359] > 1:
            return True
        else:
            return False

    def choose_direction(self, direction=None):
        for i in self.msg.ranges[0:358]:
            for j in self.msg.ranges[360:719]:
                if i < 1 and j > 1:
                    direction = 'left'
                elif i > 1 and j < 1:
                    direction = 'right'
                else:
                    direction = 'left'
        return direction

    def turn(self, var):
        if var is not None:
            direction = var
        if direction == 'left':
            self.move.angular.z += 0.1
        elif direction == 'right':
            self.move.angular.z = -0.1
        else:
            direction = 'left'


if __name__ == '__main__':
    rospy.init_node('topics_quiz_node')
    robot = Robot()
    rospy.spin()
