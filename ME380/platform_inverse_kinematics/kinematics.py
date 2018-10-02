

# -*- coding: utf-8 -*-
"""
Created on Sat Nov 14 21:10:12 2015

@author: Jak
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve,newton_krylov, anderson, broyden2
from scipy.optimize.nonlin import NoConvergence
from sympy import Plane, Point3D




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
    def servo_angle_inverse_kinematic_equations(alpha):
        # values=[servo rotation angle alpha, sx,sy,sz]
        equations = (
            # close loop x
            -1* platform_attatchment_point_bcs[0] + base_attatchment_point_bcs[0] + (servo_arm_length * np.sin(
                alpha) * np.sin(servo_axis_angle_gamma) * np.cos(servo_axis_angle_beta)) \
            - (servo_arm_length * np.cos(alpha) * np.sin(servo_axis_angle_beta)),

            # close loop y
            -1* platform_attatchment_point_bcs[1] + base_attatchment_point_bcs[1] + (servo_arm_length * np.sin(
                alpha) * np.sin(servo_axis_angle_gamma) * np.sin(servo_axis_angle_beta)) \
            + (servo_arm_length * np.cos(alpha) * np.cos(servo_axis_angle_beta)),

            # close loop z
            -1* platform_attatchment_point_bcs[2] + base_attatchment_point_bcs[2] + (servo_arm_length * np.sin(
                alpha) * np.cos(servo_axis_angle_gamma)),

            # second arm length


        )
        constraint=coupler_length ** 2 - equations[0] ** 2 - equations[1] ** 2 - equations[2] ** 2
        return constraint
    alpha= newton_krylov(servo_angle_inverse_kinematic_equations, [0.2] )[0]
    alpha=np.mod(alpha,2*np.pi) #find right hand plane version of alpha
    return alpha

def solve_inverse_kinematics(base_angles,base_radius,platform_angles,platform_radius,target_platform_position,servo_arm_length,coupler_length,gamma):
    base_attatchment_points_bcs=np.array([[base_radius*np.cos(theta), base_radius*np.sin(theta), 0] for theta in base_angles])
    platform_attatchment_points_pcs = np.array([[platform_radius * np.cos(theta), platform_radius * np.sin(theta), 0] for theta in platform_angles])
    platform_attatchment_points_bcs = find_platform_mounting_point_positions(base_attatchment_points_bcs,platform_attatchment_points_pcs,target_platform_position)
    servo_angles=[]
    for index,platform_attatchment_point in enumerate(platform_attatchment_points_bcs):
        servo_angles.append(find_servo_angle(platform_attatchment_point,base_attatchment_points_bcs[index],servo_arm_length,coupler_length,base_angles[index],gamma))
    servo_angles_degrees=np.degrees(servo_angles)
    return servo_angles

def pitch_roll_from_spherical(theta,total_angle_of_tilt):
    #theta=angle from x axis
    #total_angle_of_tile=angle from x-y plane
    rotation_vector=np.array([np.cos(theta)*np.sin(total_angle_of_tilt),np.sin(theta)*np.sin(total_angle_of_tilt),np.cos(total_angle_of_tilt)])
    roll=np.arccos(np.dot([rotation_vector[0],0,rotation_vector[2]],np.array([0,0,1]))/np.linalg.norm([rotation_vector[0],0,rotation_vector[2]]))
    pitch=np.arccos(np.dot([0,rotation_vector[1],rotation_vector[2]],np.array([0,0,1]))/np.linalg.norm([0,rotation_vector[1],rotation_vector[2]]))
    return pitch,roll

def find_range_of_motion(base_angles,base_radius,platform_angles,platform_radius,desired_angle_of_rotation,height_bounds,servo_arm_length,coupler_length, gamma, n_height_steps=10,angle_bounds_deg=(0,60),n_angle_steps=60):
    results=np.ones((n_height_steps,n_angle_steps))
    height_step=(height_bounds[1]-height_bounds[0])/n_height_steps
    angle_step=np.radians((angle_bounds_deg[1]-angle_bounds_deg[0]))/n_angle_steps
    for i in range(n_height_steps):
        current_height=height_bounds[0]+i*height_step
        print("trying angles at height {}".format(current_height))
        for j in range(n_angle_steps):
            current_angle=np.radians(angle_bounds_deg[0])+(j*angle_step)
            pos_pitch,pos_roll=pitch_roll_from_spherical(current_angle,desired_angle_of_rotation)
            try:
                solve_inverse_kinematics(base_angles,base_radius,platform_angles,platform_radius,[0,0,current_height,pos_pitch,pos_roll,0],servo_arm_length,coupler_length,gamma)
                solve_inverse_kinematics(base_angles, base_radius, platform_angles, platform_radius,
                                         [0, 0, current_height, -1*pos_pitch, -1*pos_roll, 0], servo_arm_length,
                                         coupler_length, gamma)
            except NoConvergence as e:
                results[i,j]=0
            except ValueError as e:
                results[i, j] = 0
    working_heights=[]
    for i in range(n_height_steps):
        if(all(results[i,:])):
            working_heights.append(height_bounds[0]+i*height_step)
    return working_heights


if __name__=="__main__":

    test_base_angles = np.array([np.radians(x) for x in [60, 120, 180, 240, 300, 360]])
    test_base_radius=50
    test_platform_angles = np.array([np.radians(x) for x in [30, 150, 150, 270, 270, 30]])
    test_platform_radius = 30
    test_home_height=85
    test_link_1_length=25
    test_link_2_length=100
    test_gamma=np.arctan((test_base_radius-test_platform_radius)/test_home_height)
    results=find_range_of_motion(test_base_angles,test_base_radius,test_platform_angles,test_platform_radius,np.radians(35),[85,120],test_link_1_length,test_link_2_length,test_gamma)
    print("")