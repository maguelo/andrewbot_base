#!/usr/bin/env python
import rospy
import math
from andrewbot_msgs.msg import RobotCommand
from geometry_msgs.msg import Point

robot_command = RobotCommand()
robot_command.command = [0x0d]+[0x00]*17


def calculate_quadrant_properties(x,y):
    """
    Analyze point and return quadrant, magnitude and direction
    """
    if x<0:
        quadrant = 1 if y>=0 else 4
    else:
        quadrant = 2 if y>=0 else 3
            
    magnitude = math.sqrt(math.pow(x,2) + math.pow(y,2))
    
    try:
        direction = abs(math.atan(y/x))
    except ZeroDivisionError:
        direction = math.pi/2
                
    return magnitude, direction, quadrant


def translate_point_to_velocity_cmd(x, y, v_max=255, magnitude_max=1.0):
    """
    Given a Point translate to velocity command to the robot
    """
    magnitude, direction, quadrant = calculate_quadrant_properties(x,y)

    if quadrant in [1, 4]:
        cmd_left= min(magnitude_max, magnitude)*v_max
        cmd_right = min(magnitude_max, magnitude)*v_max *math.sin(direction)

    elif quadrant in [2, 3]:
        cmd_left= min(magnitude_max, magnitude)*v_max*math.sin(direction)
        cmd_right = min(magnitude_max, magnitude)*v_max 

    else:
        raise ValueError("Unknown quadrant: {}".format(quadrant))

    if quadrant in [3,4]:
        cmd_left=-cmd_left
        cmd_right=-cmd_right

    return cmd_left, cmd_right


def move_base_callback(movement_data):
    cmd_left, cmd_right=translate_point_to_velocity_cmd(movement_data.x, movement_data.y)
    
    dir_left = 0x01
    dir_right = 0x01
    
    if cmd_left < 0:
        dir_left=0x02
    
    if cmd_right < 0:
        dir_right=0x02

    robot_command.command[1]=dir_left
    robot_command.command[2]=dir_right
    robot_command.command[3]=int(abs(cmd_left))
    robot_command.command[4]=int(abs(cmd_right))

def prepare_message():
    checksum = 0
    for v in robot_command.command[:16]:
        checksum += v

    robot_command.command[16]=(checksum >> 8) & 0xff
    robot_command.command[17]= checksum & 0xff
    

if __name__ == '__main__':
    
    
    
    try:
        rospy.init_node('andrewbot_base_control', anonymous=True)
        
        rospy.Subscriber("andrewbot/BaseCommand", Point, move_base_callback)
        base_move_pub = rospy.Publisher('andrewbot/RobotCommand', RobotCommand, queue_size=10)
        
        rate = rospy.Rate(100)  # 10 ms                                
        
        while not rospy.is_shutdown():
            prepare_message()
            base_move_pub.publish(robot_command)
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
