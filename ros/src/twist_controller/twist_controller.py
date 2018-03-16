from yaw_controller import YawController
from pid import PID
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.throttle_controller = PID(kp=0.05, ki=0.001, kd=8.5, mn=0, mx=1)

    def control(self, target_linear_velocity, target_angular_velocity, current_velocity, is_dbw_enabled, sample_time):
        error = target_linear_velocity - current_velocity
        # rospy.logwarn('error: {}'.format(error))

        throttle = self.throttle_controller.step(error, sample_time)
        steer = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_velocity)

        # Return throttle, brake, steer
        return throttle, 0.0, steer
