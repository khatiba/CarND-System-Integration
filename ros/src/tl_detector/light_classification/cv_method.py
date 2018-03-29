import numpy as np
import cv2
from styx_msgs.msg import TrafficLight
import rospy

class CVMethod():
  def __init__(self):
    self.image_no = 10000


  def predict(self, image):
    """
    image: cv2.Image (BGR)
    Reference: https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/
    """
    self.image_no = self.image_no+1
    #rospy.loginfo("image number is %s", self.image_no)
    #cv2.imwrite("imgs/img_original_"+str(self.image_no)+".png", image)

    # MedianBlur the image to handle noise
    bgr_image = cv2.medianBlur(image,3)
    #bgr_image = image

    # Convert input image to HSV
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    #cv2.imwrite("imgs/img_hsv_"+str(self.image_no)+".png", hsv_image)


    # Threshold the HSV image, keep only the red pixels
    # deliberately consider YELLOW as RED to make sure driver slow down in YELLOW
    # Also the color of the traffic light in simulator is very different to the color in rosbag
    # So extra range is created to make sure that both scenarios are considered
    carla_red_yellow_range = cv2.inRange(hsv_image,np.array([10, 40, 250]), np.array([31, 150, 255]))
    simulator_upper_red_range = cv2.inRange(hsv_image,np.array([160, 40, 200]), np.array([180, 255, 255]))
    simulator_lower_red_range = cv2.inRange(hsv_image,np.array([0, 160, 200]), np.array([9, 255, 255]))
    simulator_yellow_range = cv2.inRange(hsv_image,np.array([29, 200, 242]), np.array([31, 255, 255]))
    
    #cv2.imwrite("imgs/img_lower_red_"+str(self.image_no)+".png", carla_red_yellow_range)
    #cv2.imwrite("imgs/img_upper_red_"+str(self.image_no)+".png", simulator_upper_red_range)


    # Combine the above two image
    red_hue_image = cv2.addWeighted(carla_red_yellow_range, 1.0, simulator_upper_red_range, 1.0, 0.0)
    red_hue_image = cv2.addWeighted(red_hue_image, 1.0, simulator_lower_red_range, 1.0, 0.0)
    red_hue_image = cv2.addWeighted(red_hue_image, 1.0, simulator_yellow_range, 1.0, 0.0) # for simulator


    # Slightly blur the result to avoid false positives
    blurred_image = cv2.GaussianBlur(red_hue_image,(7,7),0)
    #blurred_image = red_hue_image
    #cv2.imwrite("imgs/img_blurred_red_"+str(self.image_no)+".png", blurred_image)

    # use HoughCircles to detect circles
    circles = cv2.HoughCircles(blurred_image,cv2.HOUGH_GRADIENT,1,100,
                            param1=50,param2=15,minRadius=4,maxRadius=40)

    # Loop over all detected circles and outline them on the original image (commented out)
    # Hough circles detects only the circular line
    # we want to detect filled circle
    # Therefore we need to calculate a filled ratio
    # If it is above the ratio, consider as a proper filled circle

    blurred_image_bgr = cv2.cvtColor(blurred_image,cv2.COLOR_GRAY2BGR)

    img_with_circle = blurred_image_bgr

    prediction = TrafficLight.UNKNOWN
    if circles is not None:
        circle_number = 0
        for current_circle in circles[0,:]:
            #rospy.loginfo("circle number is %s", circle_number)
            circle_number = circle_number + 1
            center_x = int(current_circle[0])
            center_y = int(current_circle[1])
            radius = int(current_circle[2])
            new_radius = radius + 1
            grid = cv2.inRange(blurred_image[center_y-new_radius:center_y+new_radius,center_x-new_radius:center_x+new_radius],230,255)
            #rospy.loginfo("center value is %s", blurred_image[center_y,center_x])
            #cv2.imwrite("imgs/grid_"+str(self.image_no)+"_"+str(circle_number)+".png", grid)
            area = new_radius*new_radius*4
            #rospy.loginfo("area is %s", area)
            #rospy.loginfo("center is %s,%s", center_x, center_y)
            #rospy.loginfo("radius is %s", radius)
            occupied = cv2.countNonZero(grid)
            #rospy.loginfo("occupied is %s", occupied)
            ratio = occupied/float(area)
            #rospy.loginfo("ratio is %s", ratio)
            if ratio > 0.35:
                prediction = TrafficLight.RED
                img_with_circle = cv2.circle(img_with_circle,(center_x,center_y),radius,(0,255,0),2)
                img_with_circle = cv2.putText(img_with_circle,str(ratio),(center_x,center_y),cv2.FONT_HERSHEY_SIMPLEX,2,(0,0,255),2,cv2.LINE_AA)
            else:
                img_with_circle = cv2.circle(img_with_circle,(center_x,center_y),radius,(0,255,0),2)
                img_with_circle = cv2.putText(img_with_circle,str(ratio),(center_x,center_y),cv2.FONT_HERSHEY_SIMPLEX,2,(255,0,0),2,cv2.LINE_AA)
       
        
    #cv2.imwrite("imgs/img_final_"+str(self.image_no)+".png", img_with_circle)
      
    return prediction
