from styx_msgs.msg import TrafficLight
from cv_method import CVMethod
from carla import CarlaModel
import os
import rospy

class TLClassifier(object):
    def __init__(self, scenario):
        if scenario == "cv_method":
            self.model = CVMethod()
        else:
            curr_dir = os.path.dirname(os.path.realpath(__file__))
            self.model = CarlaModel(curr_dir+"/models/frozen_inference_graph.pb")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light, BGR channel

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        result = self.model.predict(image)
        return result
