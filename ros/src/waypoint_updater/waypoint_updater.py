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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_twist_cb)
        rospy.Subscriber('traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.wp_received = False
        self.original_wps = Lane()
        self.all_wps = Lane()
        self.wp_current_start = 0
        self.wp_previous_start = 0
        self.original_speed = 0
        self.is_stopping = False
        self.is_resuming = False
        self.resuming_point = 0
        self.current_twist = TwistStamped()

        rospy.spin()

    def pose_cb(self, msg):
        if not self.wp_received:
            return

        self.wp_current_start = self.check_position(msg.pose.position)
        #rospy.loginfo("Position checked to to close to point %s", self.wp_current_start)
        if (self.wp_current_start != self.wp_previous_start) or self.is_resuming:
            num_waypoints = len(self.all_wps.waypoints)

            wps_to_send = Lane()
            wps_to_send.header = msg.header

            final_waypoints = []
            for i in range(self.wp_current_start, self.wp_current_start + LOOKAHEAD_WPS):
                final_waypoints.append(self.all_wps.waypoints[i % num_waypoints])

            # TODO: Temporarily set speed to optimize controller gains
            for i in range(len(final_waypoints)):
                self.set_waypoint_velocity(final_waypoints, i, 20.0)

            wps_to_send.waypoints = final_waypoints

            rospy.loginfo("Sending waypoints since current start is different")
            self.final_waypoints_pub.publish(wps_to_send)
            self.wp_previous_start = self.wp_current_start

    def check_position(self, pos):
        closest_point = 0
        max_distance = float('inf')
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(len(self.all_wps.waypoints)):
            current_distance = dl(pos, self.all_wps.waypoints[i].pose.pose.position)
            if current_distance < max_distance:
                closest_point = i
                max_distance = current_distance
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
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
