#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.stop_line_array = []           
        self.light_array = []
        
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.lights_received = False
        self.pos_computed = False
        self.waypoints_len= -1
        self.previous_wp = -1

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier("ssd_mobilenet_coco")
        self.listener = tf.TransformListener()



        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
	self.waypoints_len = len(waypoints.waypoints)

    def traffic_cb(self, msg):
        self.lights = msg.lights
	self.lights_received = True

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            if state == TrafficLight.RED:
                rospy.loginfo("RED or YELLOW")
            else:
                rospy.loginfo("GREEN or UNKNOWN")
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_point = 0
        max_distance = 99999
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(len(self.waypoints.waypoints)):
            current_distance = dl(pose.position,self.waypoints.waypoints[i].pose.pose.position)
            if (current_distance < max_distance):
                closest_point = i
                max_distance = current_distance
        return closest_point

    def get_closest_waypoint_with_history(self, pose, previous_point):
        closest_point = 0
        max_distance = 99999
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(previous_point, previous_point+5):
            i_wrapped = i % self.waypoints_len
            current_distance = dl(pose.position,self.waypoints.waypoints[i_wrapped].pose.pose.position)
            if (current_distance < max_distance):
                closest_point = i_wrapped
                max_distance = current_distance
        return closest_point

    

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        """
        # List of positions that correspond to the line to stop in front of for a given intersection
        """
        if (self.lights_received) and (self.waypoints) and (not self.pos_computed):
            stop_line_positions = self.config['stop_line_positions']
            for i_light in range(len(self.lights)):
                current_stop_line = Pose()
                current_stop_line.position.x = stop_line_positions[i_light][0]
                current_stop_line.position.y = stop_line_positions[i_light][1]
                current_stop_line_position = self.get_closest_waypoint(current_stop_line)
                current_light_position = self.get_closest_waypoint(self.lights[i_light].pose.pose)
                self.stop_line_array.append(current_stop_line_position)
                self.light_array.append(current_light_position)
            self.pos_computed = True
            rospy.loginfo("pos computed")

        if self.pos_computed:
            car_position = self.previous_wp
            if(self.pose):
                if (self.previous_wp == -1):
                    car_position = self.get_closest_waypoint(self.pose.pose)
                    rospy.loginfo("First previous_wp computed")
                    self.previous_wp = car_position
                else:
                    car_position = self.get_closest_waypoint_with_history(self.pose.pose,self.previous_wp)
                    self.previous_wp = car_position


            #TODO find the closest visible traffic light (if one exists)

            for i_light in range(len(self.lights)):
                current_stop_line_position = self.stop_line_array[i_light]
                current_light_position = self.light_array[i_light]
                if (current_stop_line_position > car_position):
                    distance_to_light = self.distance(self.waypoints.waypoints,car_position,current_light_position)
                    if (distance_to_light < 60):
                        light = self.lights[i_light]
                        light_wp = current_stop_line_position

            if light:
                state = self.get_light_state(light)
                return light_wp, state
            
        return -1, TrafficLight.UNKNOWN

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
