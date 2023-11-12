#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

pub_vel_left_rear_wheel = rospy.Publisher('/racecar/rear_left_velocity_controller/command', Float64, queue_size=1)
pub_vel_right_rear_wheel = rospy.Publisher('/racecar/rear_right_velocity_controller/command', Float64, queue_size=1)
pub_vel_left_front_wheel = rospy.Publisher('/racecar/front_left_velocity_controller/command', Float64, queue_size=1)
pub_vel_right_front_wheel = rospy.Publisher('/racecar/front_right_velocity_controller/command', Float64, queue_size=1)

pub_pos_left_steering_hinge = rospy.Publisher('/racecar/front_left_steering_position_controller/command', Float64, queue_size=1)
pub_pos_right_steering_hinge = rospy.Publisher('/racecar/front_right_steering_position_controller/command', Float64, queue_size=1)
pi = 3.1415926
wheel_base = 0.8
wheel_radius = 0.1
wheel_dist = 0.5
def thread_job():
    rospy.spin()

def callback(data):
    speed = data.linear.x 
    turn = -data.angular.z
    turn_abs = abs(turn)
    if turn_abs>pi/4:
        turn_abs = pi/4
    if turn_abs>0.1:
        turn_outside = math.atan(wheel_base/(wheel_dist+wheel_base/(math.tan(turn_abs))))
        if turn > 0:
            turn_left = turn_outside
            turn_right = turn_abs
        elif turn < 0:
            turn_left = -turn_abs
            turn_right = -turn_outside
    else:
        turn_left = turn_right = 0
    throttle = speed*10.0
    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(turn_left)
    pub_pos_right_steering_hinge.publish(turn_right)

def SubscribeAndPublish():
    rospy.init_node('nav_sim', anonymous=True)
    rospy.Subscriber('cmd_vel', Twist, callback,queue_size=1,buff_size=52428800)
    rospy.spin()


if __name__ == '__main__':
    try:
        SubscribeAndPublish()
    except rospy.ROSInterruptException:
        pass


########################
