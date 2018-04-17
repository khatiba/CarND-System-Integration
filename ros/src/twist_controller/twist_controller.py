from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):

    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, brake_deadband,
        accel_limit, decel_limit, vehicle_mass, wheel_radius, fuel_capacity, sample_freq):

        self.dt = 1.0/sample_freq
        self.brake_deadband = brake_deadband
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.fuel_capacity = fuel_capacity
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit


        self.yaw_controller      = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.accel_controller    = PID(kp=0.45, ki=0.02, kd=0.01, mn=decel_limit, mx=accel_limit)
        self.throttle_controller = PID(kp=1.0, ki=0.001, kd=0.10, mn=0.0, mx=0.2)
        self.lowpass_filter      = LowPassFilter(0.15, self.dt)

    def control(self, dbw_enabled, target_linear_velocity, target_angular_velocity, current_velocity):

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        vel_error = target_linear_velocity - current_velocity
        raw_accel = self.accel_controller.step(vel_error, self.dt)

        self.lowpass_filter.filt(raw_accel)
        accel = self.lowpass_filter.get()


        brake = 0.0
        throttle = 0.0
        steer = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_velocity)

        if accel > 0.0:
            throttle = self.throttle_controller.step(accel, self.dt)

        if accel <= 0.0:
            self.throttle_controller.reset()

            if abs(accel) > self.brake_deadband:
                # Assumes wheel is point mass
                brake = abs(accel) * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius

        if target_linear_velocity == 0 and current_velocity < 0.1:
            throttle = 0
            brake = 400

        return throttle, brake, steer


#from yaw_controller import YawController
#from pid import PID
#from lowpass import LowPassFilter
#import rospy
#import time
#
#GAS_DENSITY = 2.858
#ONE_MPH = 0.44704
#
#
#class Controller(object):
#
#    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, brake_deadband,
#        accel_limit, decel_limit, vehicle_mass, wheel_radius, fuel_capacity, sample_freq):
#
#        self.dt = 1.0/sample_freq
#        self.brake_deadband = brake_deadband
#        self.vehicle_mass = vehicle_mass
#        self.wheel_radius = wheel_radius
#        self.fuel_capacity = fuel_capacity
#
#        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
#
#        kp = 0.3
#        ki = 0.1
#        kd = 0.
#        mn = 0.
#        mx = 0.2
#
#        self.throttle_controller = PID(kp, ki, kd, mn, mx)
#
#        tau = 0.5
#        ts = 0.02
#        self.vel_lpf = LowPassFilter(tau,ts)
#
#        self.brake_deadband = brake_deadband
#        self.vehicle_mass = vehicle_mass
#        self.wheel_radius = wheel_radius
#        self.fuel_capacity = fuel_capacity
#        self.decel_limit = decel_limit
#        self.accel_limit = accel_limit
#
#        self.last_time = rospy.get_time()
#
#
#    def control(self, dbw_enabled, target_linear_velocity, target_angular_velocity, current_velocity):
#        
#        if not dbw_enabled:
#            self.throttle_controller.reset()
#            return 0., 0., 0.
#
#        current_velocity = self.vel_lpf.filt(current_velocity)
#
#        steer = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_velocity)
#
#        vel_error = target_linear_velocity - current_velocity
#        self.last_vel = current_velocity
#
#        current_time = rospy.get_time()
#        sample_time = current_time - self.last_time
#        self.last_time = current_time
#
#        throttle = self.throttle_controller.step(vel_error, sample_time)
#        brake = 0
#
#        if target_linear_velocity == 0 and current_velocity < 0.1:
#            throttle = 0
#            brake = 400
#
#        elif throttle < .1 and vel_error < 0:
#            throttle = 0
#            decel = max(vel_error,self.decel_limit)
#            brake = abs(decel) * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius
#        
#       # rospy.logwarn("throttle is %s", throttle)
#       # rospy.logwarn("brake is %s", brake)
#       # rospy.logwarn("steer is %s", steer)
#       # rospy.logwarn("current_vel is %s", current_velocity)
#       # rospy.logwarn("target_linear is %s", target_linear_velocity)
#       # rospy.logwarn("target_angular is %s", target_angular_velocity)
#
#        return throttle, brake, steer

