from andrewbot_utils.arduino_command import create_empty_command, prepare_message


class HeadDriver(object):
    def __init__(self, relative_movement, pitch_pos=None, yaw_pos=None, roll_pos=None):
        self.pitch_pos=pitch_pos
        self.yaw_pos=yaw_pos
        self.roll_pos=roll_pos
        
        if relative_movement:
            self.command = 0x07
        else:
            self.command = 0x08  # absolute

        self.pitch = 0
        self.yaw = 0
        self.roll = 0

    def move_pitch(self, value):
        self.pitch=value

    def move_yaw(self, value):
        self.yaw=value

    def move_roll(self, value):
        self.roll=value

    def move_head_command(self):
        command = create_empty_command()
        command[0]=self.command

        if self.pitch_pos is not None:
            command[self.pitch_pos+1]=self.pitch

        if self.yaw_pos is not None:
            command[self.yaw_pos+1]=self.yaw

        if self.roll_pos is not None:
            command[self.roll_pos+1]=self.roll

        command = prepare_message(command)
        return command