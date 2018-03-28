import numpy as np
import cv2
from styx_msgs.msg import TrafficLight

class CVMethod():
  def __init__(self):
    self.image_no = 10000


  def predict(self, image):
    """
    image: cv2.Image (BGR)
    Reference: https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/
    """
    # MedianBlur the image to handle noise
    bgr_image = cv2.medianBlur(image,5)

    # Convert input image to HSV
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image, keep only the red pixels
    # deliberately consider YELLOW as RED to make sure driver slow down in YELLOW
    # therefore increase max of lower red Hue range to 33, supposed to be 10
    lower_red_hue_range = cv2.inRange(hsv_image,np.array([0, 100, 100]), np.array([33, 255, 255]))
    upper_red_hue_range = cv2.inRange(hsv_image,np.array([160, 100, 100]), np.array([179, 255, 255]))
    
    self.image_no = self.image_no+1
    #cv2.imwrite("img_lower_red_"+str(self.image_no)+".png", lower_red_hue_range)
    #cv2.imwrite("img_upper_red_"+str(self.image_no)+".png", upper_red_hue_range)


    # Combine the above two image
    red_hue_image = cv2.addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0)

    # Slightly blur the result to avoid false positives
    blurred_image = cv2.GaussianBlur(red_hue_image,(9,9),2,2)
    #cv2.imwrite("img_blurred_red_"+str(self.image_no)+".png", blurred_image)

    # use HoughCircles to detect circles
    circles = cv2.HoughCircles(blurred_image,cv2.HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=5,maxRadius=100)

    prediction = TrafficLight.UNKNOWN
    if circles is not None:
        prediction = TrafficLight.RED
      
    return prediction
