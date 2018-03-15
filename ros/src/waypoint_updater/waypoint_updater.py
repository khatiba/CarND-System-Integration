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
        self.last_waypoints = []
        self.last_pose = None
        self.last_traffic = None
        self.last_obstacle = None

        rospy.spin()

    def pose_cb(self, msg):
        if not self.base_waypoints:
            return

        ref_x = msg.pose.position.x;
        ref_y = msg.pose.position.y;
        ref_yaw = msg.pose.orientation.z;

        final_waypoints = []
        for waypoint in self.base_waypoints.waypoints:
            shift_x = waypoint.pose.pose.position.x - ref_x
            shift_y = waypoint.pose.pose.position.y - ref_y

            new_x = shift_x * math.cos(0 - ref_yaw) - shift_y * math.sin(0 - ref_yaw)
            # new_y = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw)
            if new_x > ref_x:
                final_waypoints.append(waypoint)

        final_waypoints = final_waypoints[:200]
        for idx in range(len(final_waypoints)):
            self.set_waypoint_velocity(final_waypoints, idx, 20)

        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = final_waypoints

        # rospy.logwarn('START -------------------')
        # rospy.logwarn(len(final_waypoints))
        # rospy.logwarn('-------------------')
        # rospy.logwarn(ref_x)
        # rospy.logwarn(ref_y)
        # rospy.logwarn('-------------------')
        # rospy.logwarn(final_waypoints[0].pose.pose.position.x)
        # rospy.logwarn(final_waypoints[0].pose.pose.position.y)
        # rospy.logwarn('END -------------------')

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
