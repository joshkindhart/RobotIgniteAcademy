#!/usr/bin/env python

# -----------------------------------------------------------------------------------------

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# -----------------------------------------------------------------------------------------


class Robot():
    def __init__(self, msg):
        self.move = Twist()
        self.velocity = 0.1
        self.time = 5
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def should_proceed(self, msg):
        if msg.ranges[359] > 1.0 or msg.ranges[359] == float('inf'):
            print(msg.ranges[359], "I am good to proceed! ")
            return True
        else:
            print('Something is in my way \n')
            return False

    def move_forward(self):
        self.move.linear.x = self.velocity
        print('Proceeding! \n')

    def is_advancing(self):
        if not self.move.linear.x == 0.0:
            return False
        else:
            return True

    def stop(self):
        self.move.linear.x = 0.0
        print("Stopping \n")

    def choose_direction(self, msg, direction=None):
        for i in msg.ranges[0:240]:
            for j in msg.ranges[480:719]:
                if i > j:
                    direction = 'right'
                elif i < j:
                    direction = 'left'
        return direction

    def turn(self, direction):
        if direction == 'left':
            self.velocity = 0.1
            self.move.angular.z = self.velocity
            print('Turning {robot.choose_direction}! \n')
        elif direction == 'right':
            self.velocity = -0.1
            self.move.angular.z = self.velocity
            print('Turning {robot.choose_direction}! \n')

# -----------------------------------------------------------------------------------------


def robot_controller(msg):
    robot = Robot(msg)
    if robot.should_proceed(msg):
        robot.move_forward()
        robot.pub.publish(robot.move)
    else:
        robot.stop()
        robot.pub.publish(robot.move)

    if not robot.is_advancing():
        robot.turn(robot.choose_direction)
        robot.pub.publish(robot.move)


# -----------------------------------------------------------------------------------------


def callback(msg):
    robot_controller(msg)


def listener():
    rospy.init_node('topics_quiz')
    sub = rospy.Subscriber('/kobuki/laser/scan/', LaserScan, callback)
    rospy.spin()

# -----------------------------------------------------------------------------------------


if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            listener()
        except rospy.ROSInterruptException:
            pass
        break
