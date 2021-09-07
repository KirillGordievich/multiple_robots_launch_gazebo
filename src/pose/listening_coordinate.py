#!/usr/bin/python

import rospy
import math

from tf.msg import *
import re
from multiple_robots_launch_gazebo.msg import Pose_
from tf.transformations import euler_from_quaternion



rospy.init_node("bot_pose")

def CallBack(data):
    msg = Pose_()
    for i in range(len(data.transforms)):
     name = re.findall(r'bot\d', data.transforms[i].child_frame_id) # data.transforms[i].child_frame_id looks like "child_frame_id: "botX_tf/link_chassis"
     p = rospy.Publisher(str(name[0]) + "/pose", Pose_, queue_size=3)
     msg.x = data.transforms[i].transform.translation.x 
     msg.y = data.transforms[i].transform.translation.y
     euler = euler_from_quaternion([data.transforms[i].transform.rotation.x, data.transforms[i].transform.rotation.y, data.transforms[i].transform.rotation.z, data.transforms[i].transform.rotation.w])
     msg.yaw = euler[2]
     msg.id = re.findall(r'\d', name[0])[0]
     p.publish(msg)

def listener():
	s = rospy.Subscriber("/tf", tfMessage, CallBack)
	rospy.spin()


if __name__ == '__main__':

     listener()
