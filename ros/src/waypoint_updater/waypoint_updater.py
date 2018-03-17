#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = []

        self.curr_time = rospy.Time.now().secs
        self.target_vel = 40

        rospy.spin()

    def pose_cb(self, msg):
        if not self.base_waypoints:
            return

        num_waypoints = len(self.base_waypoints.waypoints)

        curr_pos = msg.pose.position

        min_dist = float('inf')
        closest_wpt_index = 0
        for i in range(len(self.base_waypoints.waypoints)):
            wpt_pos = self.base_waypoints.waypoints[i].pose.pose.position

            dist = math.sqrt(
                (wpt_pos.x - curr_pos.x)**2 +
                (wpt_pos.y - curr_pos.y)**2 +
                (wpt_pos.z - curr_pos.z)**2
            )
            if dist < min_dist:
                min_dist = dist
                closest_wpt_index = i

        final_waypoints = []
        for i in range(closest_wpt_index, closest_wpt_index + LOOKAHEAD_WPS):
            final_waypoints.append(self.base_waypoints.waypoints[i % num_waypoints])

        # TODO: Temporary set constant speed and stop after 20 seconds to test braking
        if msg.header.stamp.secs > self.curr_time + 20:
            self.target_vel = self.target_vel * 0.99

        for i in range(len(final_waypoints)):
            self.set_waypoint_velocity(final_waypoints, i, self.target_vel)

        lane = Lane()
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = final_waypoints

        self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints

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
