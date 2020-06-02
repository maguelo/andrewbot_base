from andrewbot_utils.arduino_command import create_empty_command, prepare_message


class BaseDriver(object):
    def __init__(self, max_velocity=255):
        self.v_max = max_velocity
        

    def _move_base_command(self, cmd_left, cmd_right):
        print cmd_left, cmd_right
        command = create_empty_command()

        dir_left = 0x01
        dir_right = 0x01

        if cmd_left < 0:
            dir_left=0x02

        if cmd_right < 0:
            dir_right=0x02

        command[0]=0x0d
        command[1]=dir_left
        command[2]=dir_right
        command[3]=int(abs(cmd_left))
        command[4]=int(abs(cmd_right))   

        command = prepare_message(command)
        return command
        
    
    def rotate_base(self, angular):
        if angular == 0:
            return None
        else: # turn
            cmd_left=  abs(angular) * self.v_max
            cmd_right = abs(angular) * self.v_max
            
            if angular >= 0: # Turn left
                cmd_left = -cmd_left
            else:           # Turn right
                cmd_right= -cmd_right

            return self._move_base_command(cmd_left, cmd_right)
        
    

    def translate_base(self, linear):
        if linear != 0.0:
            cmd_left = linear * self.v_max
            cmd_right= linear * self.v_max    
            return self._move_base_command(cmd_left, cmd_right)
        return None
    
    
