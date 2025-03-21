# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        # Initialise a publisher to publish messages to the robot base
        # We covered which topic receives messages that move the robot in the 3rd Lab Session
        self.cmdpublisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.msgpublisher = self.create_publisher(String, 'chatter', 10)
        self.rate = self.create_rate(10)  # 10 Hz


        # Initialise any flags that signal a colour has been detected (default to false)
        self.colour1_flag = False
        self.colour2_flag = False
        self.moveBackwardsFlag = False
        self.moveForwardsFlag = False
        self.stopFlag = False

        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10

        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
        self.msg1 = String()
        self.msg1.data = f'move forward'
        
        self.msg2 = String()
        self.msg2.data = f'move backward'
        
        self.msg3 = String()
        self.msg3.data = f'0000 (stop)'
        
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

        # We covered which topic to subscribe to should you wish to receive image data

    def callback(self, data):

        # Convert the received image into a opencv image
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
                print(e)
        # But remember that you should always wrap a call to this conversion method in an exception handler
        

        # Set the upper and lower bounds for the two colours you wish to identify
        #hue value = 0 to 179
        
        
        #hsv_colour1_lower = np.array([<Hue value> - self.sensitivity, 100, 100])
        #hsv_colour1_upper = np.array([<Hue value> + self.sensitivity, 255, 255])
        
        #hsv_colour2_lower = np.array([<Hue value> - self.sensitivity, 100, 100])
        #hsv_colour2_upper = np.array([<Hue value> + self.sensitivity, 255, 255])
        
        
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255]) 

        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Filter out everything but a particular colour using the cv2.inRange() method
        green_mask = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        blue_mask = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)


        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
        gb_mask = cv2.bitwise_or(green_mask, blue_mask)
        filtered_img = cv2.bitwise_and(image, image, mask=gb_mask)

        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        gcontours, _ = cv2.findContours(green_mask,mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        bcontours, _ = cv2.findContours(blue_mask,mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)


        # Loop over the contours
        if len(gcontours)>0:

            # There are a few different methods for identifying which contour is the biggest
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            gc = max(gcontours, key=cv2.contourArea)

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(gc) > 2: #<What do you think is a suitable area?>
                # Alter the value of the flag
                myColourFlag = True
                self.colour1_flag = myColourFlag
        else:
            myColourFlag = False
            self.colour1_flag = myColourFlag
                
        # Loop over the contours
        if len(bcontours)>0:

            # There are a few different methods for identifying which contour is the biggest
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            bc = max(bcontours, key=cv2.contourArea)

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(bc) > 2: #<What do you think is a suitable area?>
                # Alter the value of the flag
                myColourFlag = True
                self.colour2_flag = myColourFlag
        else:
            myColourFlag = False
            self.colour2_flag = myColourFlag

        # further away -> bigger contour area
        # closer -> smaller contour area
        #Check if a flag has been set = colour object detected - follow the colour object
        if self.colour1_flag == 1 and self.colour2_flag != 1:
            if cv2.contourArea(gc) > 25000 :
                #print(cv2.contourArea(gc))
                # Too close to object, need to move backwards
                # Set a flag to tell the robot to move backwards when in the main loop
                self.stopFlag = False
                self.moveForwardsFlag = False
                self.moveBackwardsFlag = True
                self.msgpublisher.publish(self.msg2)
                
            elif cv2.contourArea(gc) < 7000 :
                #print(cv2.contourArea(gc))
                # Too far away from object, need to move forwards
                # Set a flag to tell the robot to move forwards when in the main loop
                self.stopFlag = False
                self.moveBackwardsFlag = False
                self.moveForwardsFlag = True
                self.msgpublisher.publish(self.msg1)
            #else:
                

        # Be sure to do this for the other colour as well
        # Setting the flag to detect blue, and stop the turtlebot from moving if blue is detected
        if self.colour2_flag == 1:
            self.moveBackwardsFlag = False
            self.moveForwardsFlag = False
            self.stopFlag = True
            self.msgpublisher.publish(self.msg3)


        # Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', filtered_img)
        cv2.resizeWindow('camera_Feed',320,240)
        cv2.waitKey(3)

    def walk_forward(self):
        #Use what you learnt in lab 3 to make the robot move forwards
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2  # Forward with 0.2 m/s


        for _ in range(30):  # Stop for a brief moment
            self.cmdpublisher.publish(desired_velocity)
            self.rate.sleep()

    def walk_backward(self):
        # Use what you learnt in lab 3 to make the robot move backwards
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.2  # Backward with 0.2 m/s


        for _ in range(30):  # Stop for a brief moment
            self.cmdpublisher.publish(desired_velocity)
            self.rate.sleep()

    def stop(self):
        # Use what you learnt in lab 3 to make the robot stop
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  # Send zero velocity to stop the robot
        self.cmdpublisher.publish(desired_velocity)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    robot = Robot()
    


    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            # Publish moves
            #if found green:
            #    if robot is too close:
            #        move robot backward()
            #    else:
            #        robot walk forward()
            if robot.colour1_flag:
                if robot.moveBackwardsFlag == True:
                    robot.walk_backward()
                else:
                    robot.walk_forward()
            elif robot.colour2_flag:
                robot.stop()

    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
