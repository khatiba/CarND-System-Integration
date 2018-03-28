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

        self.light_wp_offset = 4.0
        self.wp_received = False
        self.original_wps = Lane()
        self.all_wps = Lane()
        self.wp_current_start = 0
        self.wp_previous_start = 0
        self.original_speed = 0
        self.stopping = False
        self.resuming = False
        self.resuming_point = 0
        self.current_twist = TwistStamped()
        self.last_closest_waypoint = 0

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_twist_cb)
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

        for i in range(self.last_closest_waypoint, self.last_closest_waypoint + LOOKAHEAD_WPS):# len(self.all_wps.waypoints)):
            current_distance = dl(pos, self.all_wps.waypoints[i % num_waypoints].pose.pose.position)
            if current_distance < max_distance:
                closest_point = i % num_waypoints
                max_distance = current_distance

        self.last_closest_waypoint = closest_point
        return closest_point

    def waypoints_cb(self, waypoints):
        rospy.loginfo("Waypoints received")
        if (not self.wp_received):
            self.original_wps = waypoints

        self.wp_received = True
        self.all_wps = waypoints

    def current_twist_cb(self, msg):
        rospy.loginfo("Current twist received")
        self.current_twist = msg

    def traffic_cb(self, msg):
        light_wp = msg.data

        rospy.loginfo("Current waypoint %s. traffic waypoints %s", self.wp_current_start, msg.data)

        current_speed = self.get_waypoint_velocity(self.all_wps.waypoints[self.wp_current_start])

        if light_wp >= 0 and current_speed > 0:
            dist_to_light = self.distance(self.all_wps.waypoints, self.wp_current_start, light_wp) - self.light_wp_offset
            rospy.logwarn('Braking distance to light: {}'.format(dist_to_light))

        # at the first moment detecting a red light close by
        extra_brake_wps = 50
        if light_wp >= 0 and not self.stopping:
            # rospy.loginfo("Traffic waypoints received %s", msg.data)
            # rospy.loginfo("Target velocity is %s", self.all_wps.waypoints[msg.data].twist.twist.linear.x)
            # rospy.loginfo("Red light ahead, set velocity to stop")

            wps_to_light = light_wp - self.wp_current_start
            for i in range(wps_to_light):
                delta = self.distance(self.all_wps.waypoints, self.wp_current_start, self.wp_current_start + i)
                if dist_to_light > 0:
                    new_speed = -1*(current_speed/dist_to_light) * delta + current_speed
                    self.set_waypoint_velocity(self.all_wps.waypoints, i + self.wp_current_start, new_speed)
                else:
                    self.set_waypoint_velocity(self.all_wps.waypoints, i + self.wp_current_start, -1.0)

            for j in range(wps_to_light-1, wps_to_light-1 + extra_brake_wps):
                self.set_waypoint_velocity(self.all_wps.waypoints, j + self.wp_current_start, -1.0)

            self.stopping = True

        num_resume_wps = 30
        if light_wp < 0:
            if (not self.resuming) and (self.stopping):
                rospy.logwarn("Green light now, resume velocity")
                rospy.logwarn("Target velocity is %s", self.speed_limit)
                self.stopping = False
                self.resuming = True
                self.resuming_point = self.wp_current_start + num_resume_wps

                min_resume_speed = 3.0
                dist_to_resume = self.distance(self.all_wps.waypoints, self.wp_current_start, self.resuming_point)
                for i in range(num_resume_wps):
                    delta = self.distance(self.all_wps.waypoints, self.wp_current_start, self.wp_current_start + i)
                    new_speed = ((self.speed_limit - min_resume_speed)/dist_to_resume) * delta + min_resume_speed
                    self.set_waypoint_velocity(self.all_wps.waypoints, i + self.wp_current_start, new_speed)

                self.all_wps = self.original_wps
                num_waypoints = len(self.all_wps.waypoints)
                for j in range(i, num_waypoints - num_resume_wps - 1):
                    self.set_waypoint_velocity(self.all_wps.waypoints, (j+self.wp_current_start) % num_waypoints, self.speed_limit)

            if (self.wp_current_start > self.resuming_point)and(self.resuming):
                self.resuming = False
                rospy.logwarn("Resuming done")

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

