#!/usr/bin/python

import rospy
from re import findall
from sensor_msgs.msg import LaserScan



def clbk_laser(data):
    name = findall(r'bot\d', data.header.frame_id)
    p = rospy.Publisher(name[0] + "/laser/scan", LaserScan, queue_size=2)
    p.publish(data)

def main():
    rospy.init_node('reading_laser')
    sub= rospy.Subscriber("/bot/laser/scan", LaserScan, clbk_laser)
    rospy.spin()



if __name__ == '__main__':
    main()
