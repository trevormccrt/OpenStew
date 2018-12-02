from platform_inverse_kinematics.arduino_interface import arduino_interface
from platform_inverse_kinematics.MotionPath import MotionPath
from platform_inverse_kinematics.StewartPlatform import StewartPlatform

import numpy as np

if __name__=="__main__":
    #6DOF demonstration
    stu = StewartPlatform(base_radius=116, platform_radius=128 / 2, servo_arm_length=45, coupler_length=220,
                              home_height=207,
                              base_attatchment_point_angles=np.array(
                                  [np.radians(x) for x in [60, 120, 180, 240, 300, 360]]),
                              platform_angles=np.array([np.radians(x) for x in [47.72, 132.38, 167.72, 252.28, 287.7, 12.28]]),
                              servo_pitch_angle=np.radians(np.arctan((100 - 128 / 2) / 204)),
                              servo_odd_even=[1, -1, 1, -1, 1, -1],
                              max_tilt=50,
                              max_angular_velocity=np.radians(100),
                              axis_offset=np.radians(125),
                              offset_90=np.radians(0),
                              offset_0=np.radians(0)
                              )


    path = MotionPath.from_platform_positions(stu,[[0,0,0,stu.home_height,np.radians(10),0,0],
                                                              [1,20,-20,stu.home_height,0,np.radians(10),0],
                                                              [2,-20,20,stu.home_height,0,0,np.radians(10)],
                                                              [3, 0, 0, stu.home_height-15, np.radians(10), 0, 0],
                                                              [4, 0, 0, stu.home_height+15, np.radians(-10), 0, 0]],30)

    path.csv_servo_trajcectories("6dof.csv")
    msg=path.string_servo_trajectories()
    ard = arduino_interface('/dev/ttyACM0', 115200)
    ard.send_string("start")
    ard.poll_until_message("ok")
    input()
    ard.reset_serial_buff()
    for message in msg:
        ard.send_string(message)
        ard.poll_until_message("ok")
        ard.reset_serial_buff()
    ard.send_string("end")
    ard.send_string("go")
    ard.poll_until_message("dn")


