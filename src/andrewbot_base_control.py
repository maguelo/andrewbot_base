#!/usr/bin/env python
import rospy
import math
from andrewbot_msgs.msg import RobotCommand
from geometry_msgs.msg import Twist
from andrewbot_base_driver import AndrewbotBaseDriver

robot_base_pub = rospy.Publisher('andrewbot/RobotCommand', RobotCommand, queue_size=10)
robot_base = AndrewbotBaseDriver()
robot_command = RobotCommand()

def sent_robot_command(command):    
    robot_command.command = command
    robot_base_pub.publish(robot_command)
    
def move_base(linear, angular):
    command = robot_base.rotate_base(angular)
    if command is not None: 
        sent_robot_command(command)

    command = robot_base.translate_base(linear)
    if command is not None: 
        sent_robot_command(command)

def move_base_callback(base_movement):
    move_base(base_movement.linear.x, base_movement.angular.z)
    

if __name__ == '__main__':
    try:
        rospy.init_node('andrewbot_base_control', anonymous=True)    
        rospy.Subscriber("andrewbot/BaseCommand", Twist, move_base_callback)
    
        rate = rospy.Rate(100)  # 10 ms                                
        
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
