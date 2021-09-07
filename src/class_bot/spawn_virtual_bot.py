#!/usr/bin/python

import rospy
from class_bot import Bot


if __name__ == "__main__":
    rospy.init_node("~virtual_bot", anonymous=True)
    name = rospy.get_param("~name")
    virtual_bot = Bot(name)
    virtual_bot.main()
    rospy.spin()

