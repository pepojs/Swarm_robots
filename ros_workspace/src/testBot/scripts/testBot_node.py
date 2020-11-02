#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    #rospy.loginfo('x: {}, y: {}'.format(x, y))

def callbackScan(msg):
    global pub
    global move
    print("right {}, left {}, front {}".format(msg.ranges[270], msg.ranges[90], msg.ranges[0]))
    move.linear.x = 0.1
    move.angular.z = 0
    if msg.ranges[0] < 1 or msg.ranges[5] < 1 or msg.ranges[355] < 1:
        move.linear.x = 0
        if msg.ranges[270] > 0.5:
            move.angular.z = -0.3
        elif msg.ranges[90] > 0.5:
            move.angular.z = 0.3
    if msg.ranges[0] < 0.5 or msg.ranges[5] < 0.5 or msg.ranges[355] < 0.5:
        move.linear.x = -0.1
        move.angular.z = 0
        
    rospy.loginfo("move {}".format(move.angular.x))
    pub.publish(move)
    
def main():
    global pub
    global move
    rospy.init_node('testBot')
    rospy.Subscriber("/odom", Odometry, callback)
    rospy.Subscriber("/scan", LaserScan, callbackScan)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    move = Twist()
    rospy.spin()
    

if __name__ == '__main__':
    main()
