from styx_msgs.msg import TrafficLight
from cv_method import CVMethod
import os
import rospy

class TLClassifier(object):
    def __init__(self, scenario):
        if scenario == "cv_method":
            self.model = CVMethod()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light, BGR channel

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        result = self.model.predict(image)
        return result
