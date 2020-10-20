#!/usr/bin/env python
import rospy
import math
from andrewbot_msgs.msg import RobotCommand
from geometry_msgs.msg import Twist
from head_driver import HeadDriver
from andrewbot_utils.math_utils import map_values

robot_head_pub = rospy.Publisher('andrewbot/RobotCommand', RobotCommand, queue_size=1)
robot_head = HeadDriver(relative_movement=False, yaw_pos=0, roll_pos=1)
head_command = RobotCommand()
old_head_command = None
head_command_cont =0 

def sent_robot_command(command):    
    head_command.command = command
    robot_head_pub.publish(head_command)
    
def move_head(yaw, pitch, roll):
    global old_head_command
    global head_command_cont

    yaw = int(map_values(yaw, -1.0, 1.0, 0, 180))
    robot_head.move_yaw(yaw)

    pitch = int(map_values(pitch, -1.0, 1.0, 0, 180))
    robot_head.move_pitch(pitch)

    roll = int(map_values(roll, -1.0, 1.0, 0, 180))
    robot_head.move_roll(roll)
    
    print (yaw, pitch, roll)
    head_command = robot_head.move_head_command()
    
    # if head_command == old_head_command:
    #     head_command_cont+=1

    #     if head_command_cont%10!=0:
    #         return
    # else:
    #     old_head_command = head_command
    #     head_command_cont=0

    sent_robot_command(head_command)
    
    

def move_head_callback(head_movement):
    move_head(head_movement.angular.x, head_movement.angular.y, head_movement.angular.z)
    

if __name__ == '__main__':
    try:
        rospy.init_node('andrewbot_servos_control', anonymous=True)    
        rospy.Subscriber("andrewbot/HeadCommand", Twist, move_head_callback)
    
        rate = rospy.Rate(1000)  # 10 ms                                
        
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
