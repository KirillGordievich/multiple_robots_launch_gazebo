#!/usr/bin/python

import rospy
import math
import random
import geometry_msgs.msg
import re
from sensor_msgs.msg import LaserScan
from multiple_robots_launch_gazebo.msg import pose_


class Bot:
    def __init__(self, name):
        self.name = name # name of the robot (string)
        self.id = re.findall(r'\d', name)[0] # unique number of the bot (int)

        self.status = "initialization" # Initialization, wait or move
        self.pose_msg = pose_() # [x, y, yaw, id], id is unique number of the bot

        self.state = 0
        self.state_description = 'start'

        self.rate_hz = 20 # Refresh Rate of the main method

        self.global_goal = [6.1, 1.5] # coordinates of the global goal, if you have many robots
        self.local_goal = [6.1, 1.5]
        self.initial_position = [0, 0]
        self.x = 0.0 # x coordinate of the bot
        self.y = 0.0 # y coordinate of the bot
        self.yaw = 0.0 
	self.linear_velocity = 0
	self.angular_velocity = 0 
	self.price = 0.0 # the price of the bot

	self.safe_distance = 1.3  # safe distance beetwen bots and obstacles/goal
        self.min_safe_distance = 0.8  # min safe distance beetwen bots and obstacles/goal
        self.max_linear_velocity = 0.4 # max linear velocity of the bot
        self.max_angular_velocity = 1.5
        self.distance_precision = 0.6 # if we approach the target at a distance less than this value, we should stop 
        self.yaw_precision = math.pi / 45 # +/- 2 degree allowed

        self.yaw_past_error = 0
        self.yaw_present_error = 0
        self.distance_past_error = 0      
        self.distance_present_error = 0 
        self.k_p = 1 # the coefficient for the proportional terms
        self.k_d = 0.5 # the coefficient for the derivative term

        self.list_of_bots = {self.id: [self.x, self.y, self.yaw]}
        self.regions = {'right': None, 'fright': None, 'front': None, 'fleft': None, 'left': None}

        self.geometry_msg = geometry_msgs.msg.Twist() # velocity msgs
        # register pub to send twist velocity 
        self.velocity_pub = rospy.Publisher("/" + self.name + "/cmd_vel", 
                                                  geometry_msgs.msg.Twist, queue_size=10)
        # register sub to get the bot pose
        self.pose_subscriber = rospy.Subscriber("/" + self.name + '/pose', 
                                                pose_, self.callback_pose, queue_size=6)
        # register sub to get the laser data of the bot
        self.pose_subscriber = rospy.Subscriber("/" + self.name + '/laser/scan', 
                                                LaserScan, self.callback_laser)
        # register pub to send bot coordinates to other bots
        self.coordinate_pub = rospy.Publisher("/environment", pose_, queue_size=1)

        # register sub to get coordinates from other bots
        self.coordinate_subscriber = rospy.Subscriber("/environment", pose_, 
                                                      self.callback_coordinate, queue_size=6)\
        # do some cleanup on shutdown
        rospy.on_shutdown(self.clean_shutdown)

    def main(self):
        while None in self.regions.values(): # waint until receive laser data
            rospy.sleep(1)
        rospy.loginfo('laser data is received, go')
        self.change_status('move')
        rospy.loginfo('Initialization is done')
        rospy.loginfo(self.initial_position)
        self.get_goal()
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.status == 'move':
                self.bug_0()
            elif self.status == 'wait':
                rospy.loginfo('wait')
                self.set_velocity([0, 0])
            else:
                rospy.loginfo('unknown status, not move, init or wait')
            linear, angular = self.get_velocity()
            self.geometry_msg.linear.x = linear
            self.geometry_msg.angular.z = angular
            self.velocity_pub.publish(self.geometry_msg)

            if not rospy.is_shutdown():
                rate.sleep()

    def bug_0(self):
        if self.can_head_toward_goal() is True: # go to point
            self.change_state(0)
            self.go_to_point(self.get_goal())
        else: # follow the wall
            self.follow_the_wall()
        

    def go_to_point(self, point):
        d = self.safe_distance
        min_d = self.min_safe_distance

        self.state_description = 'case 0 - go to point'
	x_goal, y_goal = point[0], point[1]
        pose = self.get_pose()
        distance = self.get_distance(x_goal, y_goal, pose[0], pose[1])
        err_yaw = self.get_err_yaw()

        rospy.loginfo_throttle(1, 'distance = %s and self.id = %s' % (distance, self.id))
        # stop if the bot1 is close to the goal
        if distance < self.distance_precision and self.id == '1':
           self.angle_past_error = 0
           linear, angular = 0, 0
           self.distance_past_error = distance
           rospy.signal_shutdown("The goal is reached by bot1")
        elif distance < d and self.id is not '1':
           self.angle_past_error = 0
           linear, angular = 0, 0
           self.distance_past_error = distance
        elif math.fabs(err_yaw) < self.yaw_precision:
            # linear = K*error + D*(present_error - past_error)
            angular = 0
            linear = self.k_p*(distance) + self.k_d*abs(distance-self.distance_past_error)
            self.yaw_past_error = 0
            self.distance_past_error = distance
	else:
            # angular = K*error + D*(present_error - past_error)
            # linear = K*error + D*(present_error - past_error)
            # PD controller does not work perfectly
	    angular = -(err_yaw*self.k_p + self.k_d*abs(err_yaw-self.yaw_past_error))
            linear = self.k_p*(distance) + self.k_d*abs(distance-self.distance_past_error)
            self.yaw_past_error = err_yaw
            self.distance_past_error = distance
        # speed must not be higher than maximum speed
        if abs(linear) > self.max_linear_velocity:
            linear = self.max_linear_velocity
        if abs(angular) > self.max_angular_velocity:
            angular = self.max_angular_velocity
        self.set_velocity([linear, angular])

    def follow_the_wall(self):
	x_goal, y_goal = self.get_goal()
        pose = self.get_pose()
        distance = self.get_distance(x_goal, y_goal, pose[0], pose[1])

        left = True # False if you want to turn rigth
        if left is True:
            k = -1
        d = self.safe_distance
        min_d = self.min_safe_distance

        if (self.regions['front'] > d and self.regions['fleft'] > d and
           self.regions['fright'] > d):
            self.state_description = 'case 1 - nothing'
            self.change_state(1)
        elif (self.regions['front'] < d and self.regions['fleft'] > d and
             self.regions['fright'] > d):
            self.state_description = 'case 2 - front'
            self.change_state(2)
        elif (self.regions['front'] > d and self.regions['fleft'] > d and
             self.regions['fright'] < d):
            self.state_description = 'case 3 - fright'
            self.change_state(3)
        elif (self.regions['front'] > d and self.regions['fleft'] < d and 
             self.regions['fright'] > d):
            self.state_description = 'case 4 - fleft'
            self.change_state(1)
        elif (self.regions['front'] < d and self.regions['fleft'] > d and
             self.regions['fright'] < d):
            self.state_description = 'case 5 - front and fright'
            self.change_state(2)
        elif (self.regions['front'] < d and self.regions['fleft'] < d and
             self.regions['fright'] > d):
            self.state_description = 'case 6 - front and fleft'
            self.change_state(2)
        elif (self.regions['front'] < d and self.regions['fleft'] < d and
             self.regions['fright'] < d):
            self.state_description = 'case 7 - front and fleft and fright'
            rospy.loginfo_throttle(1, self.regions)
            self.change_state(2)
        elif (self.regions['front'] > d and self.regions['fleft'] < d and
             self.regions['fright'] < d):
            self.state_description = 'case 8 - fleft and fright'
            self.change_state(1)
        else:
            self.state_description = 'unknown case'

        if distance < self.distance_precision and self.id == '1':
           self.angle_past_error = 0
           linear, angular = 0, 0
           self.distance_past_error = distance
           rospy.signal_shutdown("The goal is reached by bot1")
        elif distance < d and self.id is not '1':
           self.angle_past_error = 0
           linear, angular = 0, 0
           self.distance_past_error = distance

        if self.state == 1: # 'find the wall'
            if self.regions['fleft']  < min_d or self.regions['left'] < min_d:
                self.set_velocity([0.3, k*0.2])
            else:
                self.set_velocity([0.3, k*-0.4])
        if self.state == 2: # 'turn rigth or left
            if self.regions['front'] < min_d:
                self.set_velocity([0.2, k*0.5])
            else:
                self.set_velocity([0.3, k*0.4])
        if self.state == 3: # 'follow the wall'
            if self.regions['fleft']  < min_d or self.regions['left'] < min_d: # test with d and with d_min WARNING!!!!!!!!!!!!!!!!!!!!!!!!!
                self.set_velocity([0.4, k*0.6])
            else:
                self.set_velocity([0.4, k*0.0])

    def can_head_toward_goal(self):
        # return True if the bot can head toward the goal or False if it cannot
        d = self.safe_distance
        des_pos_x, des_pos_y = self.get_goal()

        desired_yaw = math.atan2(des_pos_y - self.y, des_pos_x - self.x)
        yaw = self.yaw
        err_yaw = desired_yaw - yaw

        if (math.fabs(err_yaw) < (math.pi / 6) and 
            self.regions['front'] > d):
            return True
        elif (err_yaw > 0 and 
             math.fabs(err_yaw) > (math.pi / 4) and 
             math.fabs(err_yaw) < (math.pi / 2) and 
             self.regions['left'] > d):
            return True
        elif (err_yaw < 0 and 
             math.fabs(err_yaw) > (math.pi / 4) and 
             math.fabs(err_yaw) < (math.pi / 2) and 
             self.regions['right'] > d):
            return True
        else:
            return False

    def change_state(self, state): # go to point, turn rigth/left
        if state is not self.state: # if state == self.state there is no need to change the state
            self.state = state # change the state of the bot

    def change_status(self, status): # stope, move, init etc
        if status is not self.status: # if status == self.status there is no need to change the status
            self.status = status # change the status of the bot
 
    def set_velocity(self, velocity):
        self.linear_velocity, self.angular_velocity = velocity
         
    def get_velocity(self):
        return self.linear_velocity, self.angular_velocity  

    def get_goal(self): # need to add trade function
        if int(self.id) == 1:
            self.local_goal = self.global_goal
        elif int(self.id) == 2:
            self.local_goal = self.list_of_bots['1'][0:2] # bot2 goal is coordinates of bot1
        elif int(self.id) == 3:
            self.local_goal = self.list_of_bots['2'][0:2] # bot3 goal is coordinates of bot2
	return self.local_goal

    def get_distance(self, x1, y1, x2, y2):
        # the distance between two points (X1, y1) and (x2, y2)
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def get_err_yaw(self): 
        goal_x, goal_y = self.get_goal()
        pose_x, pose_y = self.get_pose()
        desired_yaw = math.atan2(goal_y - pose_y, goal_x - pose_x)
        err_yaw = desired_yaw - self.yaw
        return err_yaw

    def get_yaw(self): # maybe change it
        # get yaw of the agent 
        yaw = self.yaw
        return self.yaw

    def get_pose(self): 
        return [self.x, self.y] # return the pose of the bot

    def callback_pose(self, data):
        if self.id == data.id:
            self.x = data.x
            self.y = data.y 
            self.yaw = data.yaw

            self.pose_msg.x = data.x
            self.pose_msg.y = data.y
            self.pose_msg.yaw = data.yaw
            self.pose_msg.id = self.id
            self.coordinate_pub.publish(self.pose_msg)
            if self.status == 'initialization':
                self.initial_position = [data.x, data.y]
        else:
            rospy.loginfo('not my pose msg!!!')

    def callback_coordinate(self, data):
	if data.id != self.id: # save coordinates of other bots
            self.list_of_bots[data.id] = [data.x, data.y, data.yaw]

    def callback_laser(self, data): # split laser data into 5 regions
        self.regions = {
            'right':  min(min(data.ranges[0:143]), 10),
            'fright': min(min(data.ranges[144:287]), 10),
            'front':  min(min(data.ranges[288:431]), 10),
            'fleft':  min(min(data.ranges[432:575]), 10),
            'left':   min(min(data.ranges[576:713]), 10),
        }

    def clean_shutdown(self):
        ''' Stop robot when shutting down '''
        rospy.loginfo('System is shutting down. Stopping %s...' % self.name)
        linear, angular  = 0, 0
        self.geometry_msg.linear.x = linear
        self.geometry_msg.angular.z = angular
        self.velocity_pub.publish(self.geometry_msg)
        rospy.loginfo("Stop")

