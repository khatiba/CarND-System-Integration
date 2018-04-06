import tensorflow as tf
from os import path
import numpy as np
from scipy import misc
from styx_msgs.msg import TrafficLight
import cv2
import rospy

class CarlaModel(object):
    def __init__(self, model_path):
        """
        The model is a pre-trained model obtained from tensorflow detection model zoo
        https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md
        We picked ssd_mobilenet_v1_coco as it is the fastest one in the table
        It is trained against COCO-dataset and will output boxes of detected class
        Class 10 is Traffic Light
        """
        self.sess = None
        self.path = model_path
        self.likelihood = 0.15
        self.coco_traffic_light_type = 10
        self.image_no = 10000
        tf.reset_default_graph()
        gd = tf.GraphDef()
        gd.ParseFromString(tf.gfile.GFile(self.path, "rb").read())
        tf.import_graph_def(gd, name="object_detection_api")
        self.sess = tf.Session()
        g = tf.get_default_graph()
        self.image = g.get_tensor_by_name("object_detection_api/image_tensor:0")
        self.boxes = g.get_tensor_by_name("object_detection_api/detection_boxes:0")
        self.scores = g.get_tensor_by_name("object_detection_api/detection_scores:0")
        self.classes = g.get_tensor_by_name("object_detection_api/detection_classes:0")

    
    def predict(self, img):

        self.image_no = self.image_no+1
        #cv2.imwrite("imgs/full_"+str(self.image_no)+".png", img)
        
        img_h, img_w = img.shape[:2]
        #rospy.loginfo("img_h,img_w is %s,%s",img_h,img_w)

        center_h = img_h // 2
        center_w = img_w // 2
        step_h = 150
        step_w = center_w - 150
        
        for h0,w0 in [(center_h, center_w),(center_h-step_h, center_w),(center_h, center_w-step_w),(center_h-step_h, center_w-step_w),(center_h, center_w+step_w),(center_h-step_h, center_w+step_w)]:

           """
           The Model has 300x300 input image size.
           We pick 6 300x300 regions of interest manually from the incoming image.
           Loop through it until a traffic light is found
           """
           grid = img[h0-150:h0+149, w0-150:w0+149, :]
           
           predicted_boxes, predicted_scores, predicted_classes = self.sess.run([self.boxes, self.scores, self.classes],
                                                        feed_dict={self.image: np.expand_dims(grid, axis=0)})
           predicted_boxes = predicted_boxes.squeeze()
           predicted_scores = predicted_scores.squeeze()
           predicted_classes = predicted_classes.squeeze()

           traffic_light = None
           h, w = grid.shape[:2]
           #cv2.imwrite("imgs/grid_"+str(self.image_no)+"_"+str(h0)+"_"+str(w0)+".png",grid)
           #rospy.loginfo("w,h is %s,%s",h0,w0)

           for i in range(predicted_boxes.shape[0]):
               box = predicted_boxes[i]
               score = predicted_scores[i]
               """
               For each boxes detected:
               We check:
               1. Trafficlight class has highest score
               2. Score is higher than a certain threshold
               3. It is a vertical rectangular shape with xy_ratio smaller than roughly 1/2
               4. Area must be large enough
               If we found one satisfying the category, we will stop searching
               """
               if score < self.likelihood: continue
               if predicted_classes[i] != self.coco_traffic_light_type: continue
               x0, y0 = box[1] * w, box[0] * h
               x1, y1 = box[3] * w, box[2] * h
               x0 = int(x0)
               x1 = int(x1)
               y0 = int(y0)
               y1 = int(y1)
               x_diff = x1 - x0
               y_diff = y1 - y0
               xy_ratio = x_diff/float(y_diff)
               #rospy.loginfo("image_no is %s", self.image_no)
               #rospy.loginfo("x,y ratio is %s",xy_ratio)
               #rospy.loginfo("score is %s",score)
               if xy_ratio > 0.48: continue            
               area = np.abs((x1-x0) * (y1-y0)) / float(w*h)
               #rospy.loginfo("area is %s",area)
               if area <= 0.001: continue
               traffic_light = grid[y0:y1, x0:x1]
               #rospy.loginfo("traffic light given")
           if traffic_light is not None: break


        if traffic_light is not None:
            #cv2.imwrite("imgs/light_"+str(self.image_no)+".png",traffic_light)
            #cv2.imwrite("imgs/full_"+str(self.image_no)+".png", img)
            #cv2.imwrite("imgs/grid_"+str(self.image_no)+".png",grid)
            """
            For each traffic light box detected
            we converted it to HSV and only look at brightness (V channel)
            Since light is assumed to be bright
            We filtered out light area and dark area, find the median height of them
            By consider the position of light and dark area, we infer light color.
            Assumed it is always Red Yellow Green vertically for traffic light setup

            Here I am consider Yellow as Red so that the car can slow down in advance
            when light turns yellow.
            """
            brightness = cv2.cvtColor(traffic_light, cv2.COLOR_BGR2HSV)[:,:,-1]
            light_h,light_w = np.where(brightness >= (brightness.max() - 5))
            light_h_mean = light_h.mean()
            dark_h,dark_w = np.where(brightness <= (brightness.max() - 50))
            dark_h_mean = dark_h.mean()
            total_h = traffic_light.shape[0]
            combined_h = (light_h_mean + (total_h - dark_h_mean))/2
            light_ratio = combined_h / total_h
            #rospy.loginfo("for light image %s, light ratio is %s",self.image_no,light_ratio)
            if light_ratio < 0.53: # A larger value to include most of YELLOW as RED
                #rospy.loginfo("image"+str(self.image_no-1)+" is RED")
                return TrafficLight.RED
            elif light_ratio > 0.60:
                #rospy.loginfo("image"+str(self.image_no-1)+" is GREEN")
                return TrafficLight.GREEN
            else:
                #rospy.loginfo("image"+str(self.image_no-1)+" is YELLOW")
                return TrafficLight.YELLOW
                
        return TrafficLight.UNKNOWN


