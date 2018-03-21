import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass,fuel_capacity,brake_deadband,decel_limit,accel_limit,wheel_radius,wheel_base,steer_ratio,max_lat_accel,max_steer_angle, min_speed):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        kp = 6.1
        ki = 0.0
        kd = 0.0
        self.pid = PID(kp, ki, kd)

    def control(self,target_twist,current_twist):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        # Calculate cte

#        rospy.loginfo("twist  cmd is %s",target_twist.twist.linear.x) 
#        rospy.loginfo("twist  cmd is %s",target_twist.twist.linear.y) 
#        rospy.loginfo("twist  cmd is %s",target_twist.twist.linear.z) 
#        rospy.loginfo("twist  cmd is %s",target_twist.twist.angular.x) 
#        rospy.loginfo("twist  cmd is %s",target_twist.twist.angular.y) 
#        rospy.loginfo("twist  cmd is %s",target_twist.twist.angular.z) 

#        rospy.loginfo("current cmd is %s",current_twist.twist.linear.x) 
#        rospy.loginfo("current cmd is %s",current_twist.twist.linear.y) 
#        rospy.loginfo("current cmd is %s",current_twist.twist.linear.z) 
#        rospy.loginfo("current cmd is %s",current_twist.twist.angular.x) 
#        rospy.loginfo("current cmd is %s",current_twist.twist.angular.y) 
#        rospy.loginfo("current cmd is %s",current_twist.twist.angular.z) 

        linear_vel = target_twist.twist.linear.x
        rospy.loginfo("linear velocity is %s", linear_vel)
        angular_vel = target_twist.twist.angular.z
        rospy.loginfo("angular velocity is %s", angular_vel)
        current_vel = current_twist.twist.linear.x
        rospy.loginfo("current velocity is %s", current_vel)
        steer_value = self.yaw_controller.get_steering(linear_vel,angular_vel,current_vel)
        rospy.loginfo("calculated steer_value is %s", steer_value)

        cte = linear_vel - current_vel
        a = self.pid.step(cte,0.2)*2
        #a = cte*20

        rospy.loginfo("calculated acceleration is %s", a)

        if (a>0):
            throttle = a
            brake = 0.
        else:
            throttle = 0.
            brake = -a*2

        return throttle, brake, steer_value
