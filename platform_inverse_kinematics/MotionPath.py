from platform_inverse_kinematics.StewartPlatform import StewartPlatform
import numpy as np
import matplotlib.pyplot as plt


class MotionPath(object):

    def __init__(self,stewart_platform,platform_angle_timeseries):
        """
           find platform motion timeseries given a bunch of 6DOF platflor position vectors
           :param platform_angle_timeseries: 6DOF platform position vector timeseries [time,x,y,z,pitch,roll,yaw]
           :param stewart_platform: stewart platform that is executing the motion
           """
        self.stewart_platform=stewart_platform
        self.platform_angle_timeseries=np.squeeze(platform_angle_timeseries)
        self.servo_position_timeseries=self._find_servo_motion()
        print()

    @classmethod
    def from_platform_angles(cls,stewart_platform,platform_angle_targets,discretization_density_ms):
        '''
        generate a motion path from a list of platform path endpoints in spherical coordinates
        :param stewart_platform: stewart platform the path will be used on
        :param platform_angle_targets: [[time_0, theta_0,direction_0 (angle from x axis)],[time_1, theta_1,direction_1]...]
        :param discretization_density_ms: output time density
        :return: MotionPath Object containing the desired path
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
        return cls(stewart_platform,positions)

    @classmethod
    def from_platform_positions(cls,stewart_platform,platform_position_targets,discretization_density_ms):
        '''
        generate a motion path from a list of platform path endpoints in 6DOF vector format
        :param stewart_platform: stewart platform the path will be used on
        :param platform_angle_targets: [[time_0,x_0,y_0,z_0,pitch_0,roll_0,yaw_0][time_1,x_1,y_1,z_1,pitch_1,roll_1,yaw_1]...]
        :param discretization_density_ms: output time density
        :return: MotionPath Object containing the desired path
        '''
        position_timeseries=[]
        position_timeseries.append(platform_position_targets[0])
        for index,position in enumerate(platform_position_targets[:-1]):
            delta=np.array(platform_position_targets[index+1])-np.array(position)
            steps=int(delta[0]/(discretization_density_ms/1000))
            all_step=delta/steps
            next=np.array(position)
            for step in range(steps):
                position_timeseries.append(np.array(position)+step*all_step)
        return cls(stewart_platform,position_timeseries)


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
        """
       plot platform and servo angular trajectories
       """
        plt.subplot(7, 1, 1)
        plt.plot(self.platform_angle_timeseries[:, 0], self.platform_angle_timeseries[:, 1])
        plt.ylabel('Platform tilt (rad)')
        plt.xlabel('time')
        plt.title('platform')
        for i in range(1, 7):
            plt.subplot(7, 1, i + 1)
            plt.plot(self.servo_position_timeseries[:, 0], self.servo_position_timeseries[:, i])
            plt.ylabel('servo {} angle (rad)'.format(i))
        plt.xlabel('time')
        plt.show()

    def csv_servo_trajcectories(self,fname):
        """
       save platform and servo angular trajectories to a csv
       """
        np.savetxt(fname,self.servo_position_timeseries,delimiter=',')

    def string_servo_trajectories(self,time_precision=6,angle_precision=4):
        """
       compile servo trajectories into a string using some floating point precision
       """
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
            if(time>15000):
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

