from StewartPlatform import StewartPlatform
import numpy as np
import matplotlib.pyplot as plt


class MotionPath(object):
    def __init__(self,position_vectors_mm,stewart_platform,discretization_density_ms):
        """
           find platform motion timeseries given a bunch of position vectors
           :param position_vectors_mm: desired ball postions, format [[x,y],[x,y]...] np.array
           :param stewart_platform: stewart platform that is executing the motion
           """
        self.position_vectors_mm=np.array(position_vectors_mm)
        self.stewart_platform=stewart_platform
        self.platform_angle_timeseries=self._find_platform_motion(discretization_density_ms)
        self.servo_position_timeseries=self._find_servo_motion()

    def _find_platform_motion(self,discretization_density_ms):
        """
           find platform motion timeseries given a bunch of position vectors
           :param discretization_density_s: time grid spacing
           """
        platform_angle_timeseries=np.empty((0,3),float)
        for index,position_vector in enumerate(self.position_vectors_mm[1:]):
            movement_vector_mm=position_vector-self.position_vectors_mm[index]
            movement_unitv=movement_vector_mm/np.linalg.norm(movement_vector_mm)
            platform_tilt_direction=np.arctan2(movement_unitv[1],movement_unitv[0]) # direction platorm needs to tilt in to complete a given motion
            motion_length_mm=np.linalg.norm(movement_vector_mm)
            timeseries=self.stewart_platform.constant_omega_motion_discretized_tilt(motion_length_mm,discretization_density_ms)
            if(not platform_angle_timeseries.size ==0):
                time_shift=np.ones((np.size(timeseries[:, 0])))*platform_angle_timeseries[-1,0]
                timeseries[:,0]=timeseries[:,0]+time_shift
            timerseries_with_angle=np.append(timeseries,np.ones((np.size(timeseries[:,0]),1))*platform_tilt_direction,axis=1)
            platform_angle_timeseries=np.append(platform_angle_timeseries,timerseries_with_angle,axis=0)
        return platform_angle_timeseries

        pass
    def _find_servo_motion(self):
        """
           find servo angles given a bunch of platform positions stored in self.platform_angle_timeseries
           """
        servo_angle_timeseries = np.empty((0, 7), float)
        for index in range(self.platform_angle_timeseries[:,0].size):
            pitch,roll=StewartPlatform.pitch_roll_from_spherical(self.platform_angle_timeseries[index,2], self.platform_angle_timeseries[index,1])
            print("pitch {}, roll {}".format(np.round(pitch,3),np.round(roll,3)))
            platform_position=[0,0,self.stewart_platform.home_height,pitch,roll,0]
            servo_angles=np.empty((1, 6), float)
            servo_angles[:]=self.stewart_platform.find_servo_positions(platform_position)
            servo_position=np.zeros((1,7))
            servo_position[0]=self.platform_angle_timeseries[index,0]
            servo_position[0,1:]=servo_angles
            servo_angle_timeseries = np.append(servo_angle_timeseries, servo_position, axis=0)
        return servo_angle_timeseries
    def _find_servo_steps(self):
        pass

if __name__=="__main__":
    stu = StewartPlatform(base_radius=100, platform_radius=128 / 2, servo_arm_length=45, coupler_length=220,
                          home_height=210,
                          base_attatchment_point_angles=np.array(
                              [np.radians(x) for x in [60, 120, 180, 240, 300, 360]]),
                          platform_angles=np.array([np.radians(x) for x in [30, 150, 150, 270, 270, 30]]),
                          servo_pitch_angle=np.radians(np.arctan((100 - 128 / 2) / 204)),
                          servo_odd_even=[1, -1, 1, -1, 1, -1],
                          max_angular_velocity=np.radians(40))
    # test=stu.constant_omega_motion_discretized_tilt(100,10)

    path=MotionPath(np.array([np.array([0,0]),np.array([50,0]),np.array([0,0]),np.array([-50,0])]),stu,10)
    plt.subplot(7,1,1)
    plt.plot(path.platform_angle_timeseries[:, 0], path.platform_angle_timeseries[:, 1])
    plt.ylabel('theta')
    plt.xlabel('time')
    plt.title('platform')
    for i in range(1,7):
        plt.subplot(7,1,i+1)
        plt.plot(path.servo_position_timeseries[:,0],path.servo_position_timeseries[:,i])
        plt.ylabel('theta')
        plt.title('servo {} at position {}'.format(i,np.round(stu.base_attatchment_points_bcs[i-1],3)))
    plt.xlabel('time')
    plt.show()
    print("")
