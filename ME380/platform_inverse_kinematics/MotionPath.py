from StewartPlatform import StewartPlatform
import numpy as np
import matplotlib.pyplot as plt


class MotionPath(object):
    # def __init__(self,position_vectors_mm,stewart_platform,discretization_density_ms):
    #     """
    #        find platform motion timeseries given a bunch of position vectors
    #        :param position_vectors_mm: desired ball postions, format [[x,y],[x,y]...] np.array
    #        :param stewart_platform: stewart platform that is executing the motion
    #        """
    #     self.position_vectors_mm=np.array(position_vectors_mm)
    #     self.stewart_platform=stewart_platform
    #     self.platform_angle_timeseries=self._find_platform_motion(discretization_density_ms)
    #     self.servo_position_timeseries=self._find_servo_motion()
    def __init__(self,stewart_platform,platform_angle_timeseries):
        """
           find platform motion timeseries given a bunch of position vectors
           :param position_vectors_mm: desired ball postions, format [[x,y],[x,y]...] np.array
           :param stewart_platform: stewart platform that is executing the motion
           """
        self.stewart_platform=stewart_platform
        self.platform_angle_timeseries=np.squeeze(platform_angle_timeseries)
        self.servo_position_timeseries=self._find_servo_motion()
        print()

    @classmethod
    def from_platform_angles(cls,stewart_platform,platform_angle_targets,discretization_density_ms):
        '''
        :param platform_angle_targets: [time, theta,direction (angle from x axis)]

        :return:
        '''
        positions=[]
        starting_pitch,starting_roll=stewart_platform.pitch_roll_from_spherical(platform_angle_targets[0][2],platform_angle_targets[0][1])
        current_pitch=starting_pitch
        current_roll=starting_roll
        current_time=platform_angle_targets[0][0]
        for position in platform_angle_targets[1:]:
            pitch,roll=stewart_platform.pitch_roll_from_spherical(position[2],position[1])
            delta_pitch=pitch-current_pitch
            delta_roll=roll-current_roll
            delta_time=position[0]-current_time
            time_steps=int(delta_time/(discretization_density_ms/1000))
            pitch_step=delta_pitch/time_steps
            roll_step=delta_roll/time_steps
            print(pitch_step)
            print(roll_step)
            for step in range(time_steps):
                current_pitch+=pitch_step
                current_roll+=roll_step
                current_time+=discretization_density_ms/1000
                positions.append([current_time,0,0,stewart_platform.home_height,current_pitch,current_roll,0])
        positions=np.array(positions)
        return MotionPath(stewart_platform,positions)




    def _find_platform_motion(self,discretization_density_ms):
        """
           find platform motion timeseries given a bunch of ball position vectors
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

    # def _find_servo_motion(self):
    #     """
    #        find servo angles given a bunch of platform positions stored in self.platform_angle_timeseries
    #        """
    #     servo_angle_timeseries = np.empty((0, 7), float)
    #     for index in range(self.platform_angle_timeseries[:,0].size):
    #         pitch,roll=StewartPlatform.pitch_roll_from_spherical(self.platform_angle_timeseries[index,2], self.platform_angle_timeseries[index,1])
    #         print("pitch {}, roll {}".format(np.round(pitch,3),np.round(roll,3)))
    #         platform_position=[0,0,self.stewart_platform.home_height,pitch,roll,0]
    #         servo_angles=np.empty((1, 6), float)
    #         servo_angles[:]=self.stewart_platform.find_servo_positions(platform_position)
    #         servo_position=np.zeros((1,7))
    #         servo_position[0]=self.platform_angle_timeseries[index,0]
    #         servo_position[0,1:]=servo_angles
    #         servo_angle_timeseries = np.append(servo_angle_timeseries, servo_position, axis=0)
    #     return servo_angle_timeseries

    def _find_servo_motion(self):
        """
           find servo angles given a bunch of platform positions stored in self.platform_angle_timeseries
           """
        servo_angle_timeseries = np.empty((0, 7), float)
        for index in range(self.platform_angle_timeseries[:,0].size):
            platform_position=self.platform_angle_timeseries[index,1:]
            servo_angles=np.array(self.stewart_platform.find_servo_positions(platform_position))
            servo_position=np.zeros((1,7))
            servo_position[0]=self.platform_angle_timeseries[index,0]
            servo_position[0,1:]=servo_angles
            servo_angle_timeseries = np.append(servo_angle_timeseries, servo_position, axis=0)
        return servo_angle_timeseries

    def plot_servo_trajectories(self):
        plt.subplot(7, 1, 1)
        plt.plot(self.platform_angle_timeseries[:, 0], self.platform_angle_timeseries[:, 1])
        plt.ylabel('Platform tilt (rad)')
        plt.xlabel('time')
        plt.title('platform')
        for i in range(1, 7):
            plt.subplot(7, 1, i + 1)
            plt.plot(self.servo_position_timeseries[:, 0], self.servo_position_timeseries[:, i])
            plt.ylabel('servo {} angle (rad)'.format(i))
            #plt.title('servo {} at position {}'.format(i, np.round(self.stewart_platform.base_attatchment_points_bcs[i - 1], 3)))
        plt.xlabel('time')
        #plt.tight_layout()
        plt.show()

    def csv_servo_trajcectories(self,fname):
        np.savetxt(fname,self.servo_position_timeseries,delimiter=',')

    def string_servo_trajectories(self,time_precision=6,angle_precision=4):
        def buffer_string_with_zeros(string,length):
            if(len(string)<length):
                n_buff=length-len(string)
                for i in range(n_buff):
                    string="{}{}".format(0,string)
            return string

        strings=[]
        for position in self.servo_position_timeseries:
            pos_str=""
            time=int(position[0]*1000)
            if(time>10000):
                raise Exception("Motion path too large to be stored by arduino")
            time=buffer_string_with_zeros(str(time),time_precision)
            pos_str+=time
            for angle in position[1:]:
                deg=int(np.rad2deg(angle))
                deg*=10
                deg_str=buffer_string_with_zeros(str(deg),angle_precision)
                pos_str+=deg_str
            strings.append(pos_str)
        return strings


# if __name__=="__main__":
#     stu = StewartPlatform(base_radius=100, platform_radius=128 / 2, servo_arm_length=45, coupler_length=220,
#                           home_height=210,
#                           base_attatchment_point_angles=np.array(
#                               [np.radians(x) for x in [60, 120, 180, 240, 300, 360]]),
#                           platform_angles=np.array([np.radians(x) for x in [30, 150, 150, 270, 270, 30]]),
#                           servo_pitch_angle=np.radians(np.arctan((100 - 128 / 2) / 204)),
#                           servo_odd_even=[1, -1, 1, -1, 1, -1],
#                           max_angular_velocity=np.radians(180))
#
#     path=MotionPath(np.array([np.array([0,0]),np.array([150,0]),np.array([0,0]),np.array([-150,0])]),stu,10)
#     path.plot_servo_trajectories()
#     path.csv_servo_trajcectories("servo_traj_180rads_150mmball.csv")
if __name__=="__main__":
    stu = StewartPlatform(base_radius=100, platform_radius=128 / 2, servo_arm_length=45, coupler_length=220,
                                                    home_height=210,
                                                    base_attatchment_point_angles=np.array(
                                                        [np.radians(x) for x in [60, 120, 180, 240, 300, 360]]),
                                                    platform_angles=np.array([np.radians(x) for x in [30, 150, 150, 270, 270, 30]]),
                                                    servo_pitch_angle=np.radians(np.arctan((100 - 128 / 2) / 204)),
                                                    servo_odd_even=[1, -1, 1, -1, 1, -1],
                                                    max_angular_velocity=np.radians(180))
    path=MotionPath.from_platform_angles(stu,[[0,0,0],[1,np.radians(30),0],[2,np.radians(30),np.radians(90)]],20)
    path.plot_servo_trajectories()