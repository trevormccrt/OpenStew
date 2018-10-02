

# -*- coding: utf-8 -*-
"""
Created on Sat Nov 14 21:10:12 2015

@author: Jak
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve




def find_platform_mounting_point_positions(base_mounting_points_bcs, platform_mounting_points_pcs, platform_position):
    """Finds leg lengths L such that the platform is in position defined by
    a = [x, y, z, alpha, beta, gamma]
    """
    #parse angles
    phi = platform_position[3]
    th = platform_position[4]
    psi = platform_position[5]

    platform_center_location_bcs=platform_position[0:3]

    #Calculate rotation matrix elements
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    cth = np.cos(th)
    sth = np.sin(th)
    cpsi = np.cos(psi)
    spsi = np.sin(psi)

    #Rotation Matrix
    Rzyx = np.array([[cpsi*cth, cpsi*sth*sphi - spsi*cphi, cpsi*sth*cphi + spsi*sphi]
                        ,[spsi*cth, spsi*sth*sphi + cpsi*cphi, spsi*sth*cphi - cpsi*sphi]
                        , [-sth, cth*sphi, cth*cphi]])

    #Hence platform sensor points with respect to the base coordinate system
    base_mounting_point_to_platform_center_point_bcs = platform_center_location_bcs - base_mounting_points_bcs
    
    #Hence orientation of platform wrt base
    
    rotated_platform_mounting_points = np.zeros(platform_mounting_points_pcs.shape)
    for i in range(6):
        rotated_platform_mounting_points[i, :] = np.dot(Rzyx, platform_mounting_points_pcs[i, :])
        
    platform_attatchment_points_bcs=base_mounting_points_bcs+base_mounting_point_to_platform_center_point_bcs + rotated_platform_mounting_points
	
    #In the IK, the leg lengths are the length of the vector (xbar+uvw)
    return platform_attatchment_points_bcs


    
def find_servo_angle(platform_attatchment_point_bcs,
                      base_attatchment_point_bcs,servo_arm_length,
                      coupler_length,servo_axis_angle_beta,servo_axis_angle_gamma):
    def servo_angle_inverse_kinematic_equations(values):
        # values=[servo rotation angle alpha, sx,sy,sz]
        alpha, sx, sy, sz = values
        equations = (
            # close loop x
            sx - platform_attatchment_point_bcs[0] + base_attatchment_point_bcs[0] + (servo_arm_length * np.sin(
                alpha) * np.sin(servo_axis_angle_gamma) * np.cos(servo_axis_angle_beta)) \
            - (servo_arm_length * np.cos(alpha) * np.sin(servo_axis_angle_beta)),

            # close loop y
            sy - platform_attatchment_point_bcs[1] + base_attatchment_point_bcs[1] + (servo_arm_length * np.sin(
                alpha) * np.sin(servo_axis_angle_gamma) * np.sin(servo_axis_angle_beta)) \
            + (servo_arm_length * np.cos(alpha) * np.cos(servo_axis_angle_beta)),

            # close loop z
            sz - platform_attatchment_point_bcs[2] + base_attatchment_point_bcs[2] + (servo_arm_length * np.sin(
                alpha) * np.cos(servo_axis_angle_gamma)),

            # second arm length
            coupler_length ** 2 - sx ** 2 - sy ** 2 - sz ** 2

        )
        return equations
    alpha= fsolve(servo_angle_inverse_kinematic_equations, np.array([0.2, np.sqrt(coupler_length**2/3),np.sqrt(coupler_length**2/3),np.sqrt(coupler_length**2/3)]))[0]
    alpha=np.arctan(np.sin(alpha)/np.cos(alpha)) #find right hand plane version of alpha
    return alpha

def solve_inverse_kinematics(base_angles,base_radius,platform_angles,platform_radius,target_platform_position,servo_arm_length,coupler_length,gamma):
    base_attatchment_points_bcs=np.array([[base_radius*np.cos(theta), base_radius*math.sin(theta), 0] for theta in base_angles])
    platform_attatchment_points_pcs = np.array([[platform_radius * np.cos(theta), platform_radius * np.sin(theta), 0] for theta in platform_angles])
    platform_attatchment_points_bcs = find_platform_mounting_point_positions(base_attatchment_points_bcs,platform_attatchment_points_pcs,target_platform_position)
    servo_angles=[]
    for index,platform_attatchment_point in enumerate(platform_attatchment_points_bcs):
        servo_angles.append(find_servo_angle(platform_attatchment_point,base_attatchment_points_bcs[index],servo_arm_length,coupler_length,base_angles[index],gamma))
    servo_angles_degrees=np.degrees(servo_angles)
    print("")

if __name__=="__main__":
    test_base_angles = np.array([np.radians(x) for x in [60, 120, 180, 240, 300, 360]])
    test_base_radius=50
    test_platform_angles = np.array([np.radians(x) for x in [30, 30, 150, 150, 270, 270]])
    test_platform_radius = 25
    test_home_height=100
    test_link_1_length=25
    test_link_2_length=150
    test_gamma=np.radians(15)
    solve_inverse_kinematics(test_base_angles,test_base_radius,test_platform_angles,test_platform_radius,[0,0,test_home_height,np.radians(30),0,0],test_link_1_length,test_link_2_length,test_gamma)

