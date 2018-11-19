from scipy.optimize import fsolve,newton_krylov, anderson, broyden2
from scipy.optimize.nonlin import NoConvergence
from datetime import datetime
import json
import numpy as np

class StewartPlatform(object):
    def __init__(self,base_radius,platform_radius,servo_arm_length,coupler_length,home_height, base_attatchment_point_angles, platform_angles,servo_pitch_angle,servo_odd_even,servo_angles=None,max_tilt=np.radians(30),max_angular_velocity=np.radians(20),axis_offset=0,offset_90=0,offset_0=0):
        """
                Creates a stewart platform object
                all params in consistent length units and radians for angles
                most params self explanitory
                :param home_height: height of platform when all servo arms and couplers are orthogonal
                :param base_attatchment_point_angles: angles of the connection point for each servo arm to servo shaft about base coordinate system
                :param platform_angles: angles of where rod end connects to platform about platform coordinate system
                :param servo_pitch_angle: angle of servo shaft with respect to base xy plane
                :param servo_odd_even: list of 1 or -1, for clockwise or counter-clockwise rotation of the servo
                :param max_tilt: angular range of motion of this stewart platform about some arbitrary axis at home_height
                :param max_angular_velocity: maximum angular velocity platform can move at
\
                """
        self.base_radius=base_radius
        self.platform_radius=platform_radius
        self.servo_arm_length=servo_arm_length
        self.coupler_length=coupler_length
        self.home_height=home_height
        self.base_attatchment_point_angles=np.array(base_attatchment_point_angles)
        self.platform_angles=np.array(platform_angles)
        self.servo_pitch_angle=servo_pitch_angle
        self.servo_odd_even=np.array(servo_odd_even)

        if servo_angles is None:
            self.servo_angles=base_attatchment_point_angles
        self.base_attatchment_points_bcs=np.array([[self.base_radius*np.cos(theta), base_radius*np.sin(theta), 0] for theta in self.base_attatchment_point_angles])
        self.platform_attatchment_points_pcs=np.array([[self.platform_radius * np.cos(theta), platform_radius * np.sin(theta), 0] for theta in self.platform_angles])

        self.max_tilt=max_tilt
        self.max_angular_velocity=max_angular_velocity
        self.axis_offset=axis_offset
        self.offset_90=offset_90
        self.offset_0=offset_0

    def tranlate_axis(self):
        pass

    def transform_platform_attatchment_points(self,platform_position):
        """
            transforms platform mounting points from platform coordinate system to base coordinate system for a given platform position

            :param platform_position: platform position as [x,y,z,pitch,roll,yaw]
            :returns: t
            :raises keyError: raises an exception
            """
        # parse angles
        phi = platform_position[3]
        th = platform_position[4]
        psi = platform_position[5]

        platform_center_location_bcs = platform_position[0:3]

        # Calculate rotation matrix elements
        cphi = np.cos(phi)
        sphi = np.sin(phi)
        cth = np.cos(th)
        sth = np.sin(th)
        cpsi = np.cos(psi)
        spsi = np.sin(psi)

        # Rotation Matrix
        Rzyx = np.array([[cpsi * cth, cpsi * sth * sphi - spsi * cphi, cpsi * sth * cphi + spsi * sphi]
                            , [spsi * cth, spsi * sth * sphi + cpsi * cphi, spsi * sth * cphi - cpsi * sphi]
                            , [-sth, cth * sphi, cth * cphi]])

        # Hence platform sensor points with respect to the base coordinate system
        base_mounting_point_to_platform_center_point_bcs = platform_center_location_bcs - self.base_attatchment_points_bcs

        # Hence orientation of platform wrt base

        rotated_platform_mounting_points = np.zeros(self.platform_attatchment_points_pcs.shape)
        for i in range(6):
            rotated_platform_mounting_points[i, :] = np.dot(Rzyx, self.platform_attatchment_points_pcs[i, :])

        platform_attatchment_points_bcs = self.base_attatchment_points_bcs + base_mounting_point_to_platform_center_point_bcs + rotated_platform_mounting_points
        # In the IK, the leg lengths are the length of the vector (xbar+uvw)
        return platform_attatchment_points_bcs

        pass

    def find_servo_positions(self,platform_position):
        """
            Find servo angles for a given 6DOF platform position

            :param platform_position: platform position as [x,y,z,pitch,roll,yaw]
            :returns: 6 servo angles corresponding to the given platform position
            """

        def find_servo_angle(servo):
            """
                Finds a servo angle via numerical soloution of inverse kinematic equations

                :param servo: which servo to solve
                :returns: servo angle in radians
                """
            def servo_angle_inverse_kinematic_equations(alpha):
                equations = (
                    # close loop x
                    (-1 * (transformed_platform_attatchment_points[servo])[0]) + (self.base_attatchment_points_bcs[servo])[0] - (
                                self.servo_arm_length * np.sin(
                            alpha) * np.sin(self.servo_pitch_angle) * np.cos(self.servo_angles[servo]))
                    - (self.servo_arm_length * np.cos(alpha) * np.sin(self.servo_angles[servo])),

                    # close loop y
                    (-1 * (transformed_platform_attatchment_points[servo])[1]) + (self.base_attatchment_points_bcs[servo])[1] - (
                                self.servo_arm_length * np.sin(
                            alpha) * np.sin(self.servo_pitch_angle) * np.sin(self.servo_angles[servo]))
                    + (self.servo_arm_length * np.cos(alpha) * np.cos(self.servo_angles[servo])),

                    # close loop z
                    (-1 * (transformed_platform_attatchment_points[servo])[2]) + (self.base_attatchment_points_bcs[servo])[2] + (
                                self.servo_arm_length * np.sin(
                            alpha) * np.cos(self.servo_pitch_angle)),

                    # second arm length

                )
                constraint = self.coupler_length ** 2 - equations[0] ** 2 - equations[1] ** 2 - equations[2] ** 2
                return constraint

            if (self.servo_odd_even[servo] == -1):
                init_guess = np.pi
            else:
                init_guess = 0
            alpha = newton_krylov(servo_angle_inverse_kinematic_equations, [init_guess])[0]
            #alpha = np.mod(alpha, 2 * np.pi)
            if(alpha<0):
                alpha+=2*np.pi
            return alpha

        transformed_platform_attatchment_points=self.transform_platform_attatchment_points(platform_position)
        servo_angles=[]
        for servo in range(len(self.servo_angles)):
            servo_angles.append(find_servo_angle(servo))
        return servo_angles

    def constant_omega_motion_discretized_tilt(self, motion_length_mm,discretization_time_ms):
        """
            Finds a servo angle via numerical soloution of inverse kinematic equations

            :param servo: which servo to solve
            :returns: servo angle in radians
            """
        t_ramp = self.max_tilt / self.max_angular_velocity
        def theta_long(move_time_s,time):
            term_1=self.max_angular_velocity * time * (1 - np.heaviside(time - t_ramp,1))
            term_2=self.max_angular_velocity * t_ramp * (np.heaviside(time - t_ramp,1) - np.heaviside(time - (move_time_s - t_ramp),1))
            term_3=(-1*self.max_angular_velocity * (time-(solve_time-t_ramp/2)) + self.max_angular_velocity * t_ramp) * (np.heaviside(time - (move_time_s - t_ramp),1) - np.heaviside(time - move_time_s,1))
            return term_1+term_2+term_3

        def theta_short(move_time_s,time):
            term_1=self.max_angular_velocity*time*(1-np.heaviside(time-(move_time_s/2),1))
            term_2=((-1*self.max_angular_velocity*(time-move_time_s/2)+(self.max_angular_velocity*move_time_s/2)))*(np.heaviside(time-(move_time_s/2),0))
            val=term_1+term_2

            return val


        motion_length_m=motion_length_mm/(2*1000)

        short_time_s=(2*(7**(2.0/3.0))*(motion_length_m**(1.0/3.0)))/(7*(self.max_angular_velocity**(1.0/3.0)))
        long_time_s=((self.max_tilt**3 + (8*motion_length_m*self.max_angular_velocity**2)/7)**(1/2) + self.max_tilt**(3/2))/(2*self.max_tilt**(1/2)*self.max_angular_velocity)
        if short_time_s < 2*t_ramp:
            solve_time=short_time_s
            motion_calculator=theta_short
        else:
            solve_time=long_time_s
            motion_calculator=theta_long
        times=np.arange(0,solve_time,discretization_time_ms/1000.0)
        times_2=np.arange(solve_time+discretization_time_ms/1000,2*solve_time+(discretization_time_ms/1000),discretization_time_ms/1000)
        angles=np.array([motion_calculator(solve_time,step) for step in times])
        time_series=np.vstack((np.column_stack((times,angles)),np.column_stack((times_2,-1*np.flip(angles)))))

        return time_series

    @staticmethod
    def find_range_of_motion(stewart_platform,desired_angle_of_rotation,height_bounds, n_height_steps=20, angle_bounds_deg=(0, 70), n_angle_steps=5):
        """
            Finds platform heights at which the given stewart platform can tilt to desired_angle_of_rotation about an arbitrary axis

            :param height_bounds: domain of heights to try
            :param n_height_steps: how many heights to try inside domain
            :param angle_bounds_deg: domain of direcion of tilt to try (about z axis, measured from x axis)
            :param n_angle_steps: how many rotations to try in domain
            :raises NoConvergence or ValueError: if no servo position exists(platform position is not possible)
            """
        all_results = {
            "tilt_magnitude": desired_angle_of_rotation,
            "height_results":[]
        }
        height_step = (height_bounds[1] - height_bounds[0]) / n_height_steps
        angle_step = np.radians((angle_bounds_deg[1] - angle_bounds_deg[0])) / n_angle_steps

        for i in range(n_height_steps):
            current_height = height_bounds[0] + i * height_step
            print("trying angles at height {}".format(current_height))

            height_results={
                "height":current_height,
                "results":[]
            }
            angle_results=[]
            all = 1
            for j in range(n_angle_steps):
                current_angle = np.radians(angle_bounds_deg[0]) + (j * angle_step)
                result={
                    "tilt_direction":current_angle,
                    "positive_servo_angles":[],
                    "negative_servo_angles":[]
                }
                pos_pitch, pos_roll = StewartPlatform.pitch_roll_from_spherical(current_angle, desired_angle_of_rotation)
                try:
                    result["positive_servo_angles"]=stewart_platform.find_servo_positions([0, 0, current_height, pos_pitch, pos_roll, 0])
                    result["negative_servo_angles"]=stewart_platform.find_servo_positions([0, 0, current_height, -1*pos_pitch, -1* pos_roll, 0])
                    print("soln found height {} angle {}".format(current_height,current_angle))
                except NoConvergence as e:
                    result["positive_servo_angles"]=0
                    result["negative_servo_angles"]=0
                    all=0
                except ValueError as e:
                    result["positive_servo_angles"] = 0
                    result["negative_servo_angles"] = 0
                    all=0
                angle_results.append(result)
            if(all):
                height_results["results"]=angle_results
                all_results["height_results"].append(height_results)
        with open('range_of_motion_{}.json'.format(datetime.now().strftime("%Y%m%d%H%M%S")), 'w') as outfile:
            json.dump(all_results, outfile)
        return all_results


    def pitch_roll_from_spherical(self,theta, total_angle_of_tilt):
        """
            find pitch and roll components of an arbitrary platform tilt
            :param theta: direction of tilt (about z axis, measured from x axis)
            :param total_angle_of_tilt: angle of tilt to find pitch and roll for
            """
        # theta=angle from x axis
        # total_angle_of_tile=angle from x-y plane
        theta=theta+self.axis_offset

        rotation_vector = np.array(
            [np.cos(theta) * np.sin(total_angle_of_tilt), np.sin(theta) * np.sin(total_angle_of_tilt),
             np.cos(total_angle_of_tilt)]) #platform normal vector
        offset_0_vector=np.array(
            [np.cos(self.axis_offset) * np.sin(self.offset_0), np.sin(self.axis_offset) * np.sin(self.offset_0),
             np.cos(self.offset_0)])
        offset_90_vector = np.array(
            [np.cos(self.axis_offset-np.pi/2) * np.sin(self.offset_90), np.sin(self.axis_offset-np.pi/2) * np.sin(self.offset_90),
             np.cos(self.offset_90)])
        roll_nom=np.arctan2(rotation_vector[0], rotation_vector[2])
        pitch_nom=np.arctan2(rotation_vector[1], rotation_vector[2])
        roll_0 = np.arctan2(offset_0_vector[0], offset_0_vector[2])
        pitch_0 = np.arctan2(offset_0_vector[1], offset_0_vector[2])
        roll_90 = np.arctan2(offset_90_vector[0], offset_90_vector[2])
        pitch_90 = np.arctan2(offset_90_vector[1], offset_90_vector[2])
        roll_total=roll_nom-roll_0-roll_90
        pitch_total=pitch_nom-pitch_0-pitch_90
        #roll = np.arccos(np.dot([rotation_vector[0], 0, rotation_vector[2]], np.array([0, 0, 1])) / np.linalg.norm(
         #   [rotation_vector[0], 0, rotation_vector[2]]))
        #pitch = np.arccos(np.dot([0, rotation_vector[1], rotation_vector[2]], np.array([0, 0, 1])) / np.linalg.norm(
          #  [0, rotation_vector[1], rotation_vector[2]]))
        return pitch_total, roll_total



if __name__=="__main__":
    stu=StewartPlatform(base_radius=100,platform_radius=128/2,servo_arm_length=45,coupler_length=220,home_height=204,
                        base_attatchment_point_angles=np.array([np.radians(x) for x in [60, 120, 180, 240, 300, 360]]),
                        platform_angles=np.array([np.radians(x) for x in [30, 150, 150, 270, 270, 30]]),
                        servo_pitch_angle=np.radians(np.arctan((100-128/2)/204)),
                        servo_odd_even=[1,-1,1,-1,1,-1])
    range_of_motion=StewartPlatform.find_range_of_motion(stu,np.radians(35),[190,240])
    print("")



