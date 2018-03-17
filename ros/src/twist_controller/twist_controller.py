from yaw_controller import YawController
from pid import PID
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):

    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, brake_deadband,
        accel_limit, decel_limit, vehicle_mass, wheel_radius, fuel_capacity):

        self.brake_deadband = brake_deadband
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.fuel_capacity = fuel_capacity

        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.accel_controller = PID(kp=10, ki=0.05, kd=0.1, mn=decel_limit, mx=accel_limit)
        self.throttle_controller = PID(kp=1, ki=0.05, kd=0.5, mn=0, mx=1)

    def control(self, target_linear_velocity, target_angular_velocity, current_velocity, is_dbw_enabled, sample_time):
        vel_error = target_linear_velocity - current_velocity
        accel = self.accel_controller.step(vel_error, sample_time)

        brake = 0.0
        throttle = 0.0
        steer = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_velocity)

        if accel > 0.0:
            throttle = self.throttle_controller.step(accel, sample_time)

        if accel <= 0.0:
            self.throttle_controller.reset()

            if abs(accel) > self.brake_deadband:
                # Assumes wheel is point mass
                brake = abs(accel) * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius 

        # Return throttle, brake, steer
        return throttle, brake, steer
