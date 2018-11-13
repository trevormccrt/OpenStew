from serial import Serial
import time
from StewartPlatform import StewartPlatform
from MotionPath import MotionPath
import numpy as np
import threading
message=''
class arduino_interface():
    def __init__(self,port,baud_rate):
        self.ser=Serial(port,baud_rate)
        thread = threading.Thread(target=self.non_blocking_read)
        thread.start()
        self.last_serial_string=None

    def non_blocking_read(self):
        while(True):
            if(self.ser.in_waiting>0):
                data_str = self.ser.readline().decode().strip('\r\n')
                self.last_serial_string=data_str# read the bytes and convert from binary array to ASCII
                print(data_str) # print the incoming string without putting a new-line ('\n') automatically after every print()


    def poll_until_message(self,message):
        while(self.last_serial_string!=message):
            time.sleep(0.001)

    def reset_serial_buff(self):
        self.last_serial_string=None

    def send_string(self,message):
        print("Sending {}".format(message))
        message+="\r\n"
        self.ser.write(message.encode())



if __name__=="__main__":

    stu = StewartPlatform(base_radius=100, platform_radius=128 / 2, servo_arm_length=45, coupler_length=220,
                          home_height=210,
                          base_attatchment_point_angles=np.array(
                              [np.radians(x) for x in [60, 120, 180, 240, 300, 360]]),
                          platform_angles=np.array([np.radians(x) for x in [30, 150, 150, 270, 270, 30]]),
                          servo_pitch_angle=np.radians(np.arctan((100 - 128 / 2) / 204)),
                          servo_odd_even=[1, -1, 1, -1, 1, -1],
                          max_angular_velocity=np.radians(180))

    path = MotionPath(np.array([np.array([0, 0]), np.array([150, 0]), np.array([0, 0]), np.array([-150, 0])]), stu, 10)
    msg=path.string_servo_trajectories()
    ard = arduino_interface('/dev/ttyACM2', 115200)
    ard.send_string("start")
    ard.poll_until_message("ok")
    ard.reset_serial_buff()
    for message in msg:
        ard.send_string(message)
        ard.poll_until_message("ok")
        ard.reset_serial_buff()
    ard.send_string("end")
    input()
    st_time=time.time()
    ard.send_string("go")
    ard.poll_until_message("dn")
    print(time.time()-st_time)