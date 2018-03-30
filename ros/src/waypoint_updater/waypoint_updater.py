#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 75 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.speed_limit = rospy.get_param('/waypoint_loader/velocity', 15) * 5/18  # kph -> m/s

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.light_wp_offset = 4
        self.wp_received = False
        self.all_wps = Lane()
        self.wp_current_start = 0
        self.wp_previous_start = 0
        self.stopping = False
        self.resuming = False
        self.resuming_point = 0
        self.last_closest_wp = 0

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('traffic_waypoint', Int32, self.traffic_cb)

        rospy.spin()

    def pose_cb(self, msg):
        if not self.wp_received:
            return

        self.wp_current_start = self.check_position(msg.pose.position)
        #rospy.loginfo("Position checked to to close to point %s", self.wp_current_start)
        if (self.wp_current_start != self.wp_previous_start) or self.resuming:
            num_waypoints = len(self.all_wps.waypoints)

            wps_to_send = Lane()
            wps_to_send.header = msg.header

            final_waypoints = []
            for i in range(self.wp_current_start, self.wp_current_start + LOOKAHEAD_WPS):
                final_waypoints.append(self.all_wps.waypoints[i % num_waypoints])

            wps_to_send.waypoints = final_waypoints

            rospy.loginfo("Sending waypoints since current start is different")
            self.final_waypoints_pub.publish(wps_to_send)
            self.wp_previous_start = self.wp_current_start

    def check_position(self, pos):
        num_waypoints = len(self.all_wps.waypoints)

        closest_point = 0
        max_distance = float('inf')
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        for i in range(self.last_closest_wp, self.last_closest_wp + LOOKAHEAD_WPS):
            current_distance = dl(pos, self.all_wps.waypoints[i % num_waypoints].pose.pose.position)
            if current_distance < max_distance:
                closest_point = i % num_waypoints
                max_distance = current_distance

        self.last_closest_wp = closest_point
        return closest_point

    def waypoints_cb(self, waypoints):
        rospy.loginfo("Waypoints received")
        self.wp_received = True
        self.all_wps = waypoints

    def traffic_cb(self, msg):
        if self.wp_received:
            light_wp = msg.data
            num_waypoints = len(self.all_wps.waypoints)

            if light_wp >= 0:
                rospy.loginfo("Current waypoint %s. Traffic waypoint %s, Stop line waypoint %s",
                    self.wp_current_start, light_wp, light_wp - self.light_wp_offset)

            # at the first moment detecting a red light close by
            extra_brake_wps = 25
            if light_wp >= 0 and not self.stopping:
                rospy.loginfo("Red light ahead, begin stop")

                current_speed = self.get_waypoint_velocity(self.all_wps.waypoints[self.wp_current_start])
                target_speed = -1.0 # hold the break at a stop
                target_wp = light_wp - self.light_wp_offset # Stop just behind the line

                self.linearly_change_speed(current_speed, target_speed, target_wp)

                for j in range(target_wp, target_wp + extra_brake_wps):
                    self.set_waypoint_velocity(self.all_wps.waypoints, j % num_waypoints, target_speed)

                self.stopping = True

            num_resume_wps = 30
            if light_wp < 0:
                if (not self.resuming) and (self.stopping):
                    rospy.loginfo("Green light now, target velocity is %s", self.speed_limit)
                    self.stopping = False
                    self.resuming = True
                    self.resuming_point = self.wp_current_start + num_resume_wps

                    self.linearly_change_speed(3.0, self.speed_limit, self.resuming_point)

                    for j in range(num_resume_wps - 1 , num_waypoints - num_resume_wps - 1):
                        self.set_waypoint_velocity(self.all_wps.waypoints, (j+self.wp_current_start) % num_waypoints, self.speed_limit)

                if (self.wp_current_start > self.resuming_point)and(self.resuming):
                    self.resuming = False
                    rospy.logwarn("Resuming done")

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    # Apply y = m*x + b. Given initial speed, target speed, current_wp, target_wp
    def linearly_change_speed(self, current_speed, target_speed, target_wp):
        num_waypoints = len(self.all_wps.waypoints)
        target_distance = max(self.distance(self.all_wps.waypoints, self.wp_current_start, target_wp), 0.01)
        wps_to_target = target_wp - self.wp_current_start
        slope = (target_speed - current_speed) / target_distance

        for i in range(wps_to_target):
            delta = self.distance(self.all_wps.waypoints, self.wp_current_start, self.wp_current_start + i)
            next_speed = slope * delta + current_speed
            next_wp = (self.wp_current_start + i) % num_waypoints
            self.set_waypoint_velocity(self.all_wps.waypoints, next_wp, next_speed)

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        num_waypoints = len(waypoints)
        rospy.loginfo("%s out of %s",num_waypoints,waypoint % num_waypoints)
        waypoints[waypoint % num_waypoints].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        num_waypoints = len(waypoints)
        wp1 = wp1 % num_waypoints
        wp2 = wp2 % num_waypoints
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i%num_waypoints].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

