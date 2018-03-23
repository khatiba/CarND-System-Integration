#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped

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

LOOKAHEAD_WPS = 150 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = []

        self.curr_time = rospy.Time.now().secs
        self.target_speed = 20

        self.current_velocity = None
        self.light_wp = -1

        self.is_stopping = False
        self.stop_timeout = 0
        self.light_wp_offset = 4

        rospy.spin()

    def pose_cb(self, msg):
        if not self.base_waypoints:
            return

        num_waypoints = len(self.base_waypoints.waypoints)

        curr_pos = msg.pose.position

        min_dist = float('inf')
        closest_wpt_index = 0
        for i in range(num_waypoints):
            wpt_pos = self.base_waypoints.waypoints[i].pose.pose.position

            dist = math.sqrt(
                (wpt_pos.x - curr_pos.x)**2 +
                (wpt_pos.y - curr_pos.y)**2 +
                (wpt_pos.z - curr_pos.z)**2
            )
            if dist < min_dist:
                min_dist = dist
                closest_wpt_index = i

        target_speed = 20

        final_waypoints = []
        for i in range(closest_wpt_index, closest_wpt_index + LOOKAHEAD_WPS):
            final_waypoints.append(self.base_waypoints.waypoints[i % num_waypoints])

        curr_wp_speed = self.get_waypoint_velocity(self.base_waypoints.waypoints[closest_wpt_index])

        dist_to_light = self.distance(self.base_waypoints.waypoints, closest_wpt_index, self.light_wp) - self.light_wp_offset

        if self.light_wp > 0 and dist_to_light < 40:
            self.is_stopping = True

            # rospy.logwarn('Braking dist to light: {}'.format(dist_to_light))
            # linearly decrease speed, eg: new_speed = - (curr_wp_speed / dist_to_light) * i + current_wp_speed
            for i in range(len(final_waypoints)):
                if dist_to_light > 0:
                    new_speed = -1 * i * (curr_wp_speed / dist_to_light) + curr_wp_speed
                    self.set_waypoint_velocity(final_waypoints, i, new_speed)
                else:
                    self.set_waypoint_velocity(final_waypoints, i, -1)
        else:
            self.is_stopping = False

        if self.is_stopping:
            self.stop_timeout = self.stop_timeout + 1
            if self.stop_timeout > 500:
                rospy.logwarn('Starting again')
                self.is_stopping = False
                self.stop_timeout = 0

        if not self.is_stopping:
            for i in range(len(final_waypoints)):
                self.set_waypoint_velocity(final_waypoints, i, target_speed)

        lane = Lane()
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = final_waypoints

        self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.light_wp = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def current_velocity_cb(self, msg):
        self.current_velocity = msg

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
