#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped,TwistStamped
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

        rospy.Subscriber('/current_velocity', TwistStamped, self.vel_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('traffic_waypoint', Int32, self.traffic_cb)



        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
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

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        #rospy.loginfo("Pose received")
        if (self.wp_received):
            self.wp_current_start = self.check_position(msg.pose.position)
            #rospy.loginfo("Position checked to to close to point %s", self.wp_current_start)
            if ((self.wp_current_start != self.wp_previous_start))or(self.resuming):
                wps_to_send = Lane()
                wps_to_send.header = msg.header
                wps_to_send.waypoints = self.all_wps.waypoints[self.wp_current_start:self.wp_current_start+LOOKAHEAD_WPS-1]
                rospy.loginfo("Sending waypoints since current start is different")
                self.final_waypoints_pub.publish(wps_to_send)
                self.wp_previous_start = self.wp_current_start
                
    def vel_cb(self,vel_msg):
        rospy.loginfo("Current vel received")
        self.current_twist = vel_msg
            
        
    def check_position(self,pos):
        closest_point = 0
        max_distance = 99999
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(len(self.all_wps.waypoints)):
            current_distance = dl(pos,self.all_wps.waypoints[i].pose.pose.position)
            if (current_distance < max_distance):
                closest_point = i
                max_distance = current_distance
        return closest_point

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        rospy.loginfo("Waypoints received")
        if (not self.wp_received):
            self.original_wps = waypoints
        self.wp_received = True
        self.all_wps = waypoints 

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        rospy.loginfo("Current waypoint is %s", self.wp_current_start)
        rospy.loginfo("Traffic waypoints received %s", msg.data)
        
        # at the first moment detecting a red light close by
        if ((msg.data >= 0)and(not self.stopping)):
            rospy.loginfo("Traffic waypoints received %s", msg.data)
            rospy.loginfo("Target velocity is %s", self.all_wps.waypoints[msg.data].twist.twist.linear.x)
            rospy.loginfo("Red light ahead, set velocity to stop")
            self.original_speed = self.current_twist.twist.linear.x
            rospy.loginfo("current speed is %s", self.original_speed)
            braking_in_action = (msg.data - self.wp_current_start+1)//2
            rospy.loginfo("waypoints before stopping is %s", braking_in_action)
            for i in range(braking_in_action):
                rospy.loginfo("i is %s", i)
                self.set_waypoint_velocity(self.all_wps.waypoints,i+self.wp_current_start,self.original_speed*(braking_in_action-i-1)/(braking_in_action-1))
                #self.set_waypoint_velocity(self.all_wps.waypoints,i+self.wp_current_start,0.)
            for i in range(10+braking_in_action):
                self.set_waypoint_velocity(self.all_wps.waypoints,i+braking_in_action+self.wp_current_start,0.)
            self.stopping = True
        else:
            if (msg.data < 0):
                if ((not self.resuming) and (self.stopping)):
                    rospy.loginfo("Green light now, resume velocity")
                    self.stopping = False
                    self.resuming = True
                    self.resuming_point = self.wp_current_start+50
                    resuming_in_action = 50
                    #target_speed = self.original_wps.waypoints[self.wp_current_start+20].twist.twist.linear.x
                    target_speed = 100/9
                    rospy.loginfo("Target velocity is %s", target_speed)
                    own_speed = self.current_twist.twist.linear.x
                    rospy.loginfo("Own velocity is %s", own_speed)
                    self.all_wps = self.original_wps
                    for i in range(resuming_in_action):
                        rospy.loginfo("i is %s", i)
                        #self.set_waypoint_velocity(self.all_wps.waypoints,i+self.wp_current_start,own_speed+(target_speed-own_speed)*(i+1)/(resuming_in_action))
                        self.set_waypoint_velocity(self.all_wps.waypoints,i+self.wp_current_start,target_speed)
                    for i in range(110):
                        self.set_waypoint_velocity(self.all_wps.waypoints,i+self.wp_current_start+resuming_in_action,target_speed)
                if (self.wp_current_start > self.resuming_point)and(self.resuming):
                    self.resuming = False
                    rospy.loginfo("Resuming done")


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity
        rospy.loginfo("setting waypoint %s to %s", waypoint, velocity)

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
