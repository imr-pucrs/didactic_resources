#!/usr/bin/env python

"""
This script drives a simulated Turtlebot to follow lines around a Gazebo simulation.
The course containing lines to Turtlebot follow is illustrated in `tracking.png` file. 

The script uses ROSpy and OpenCV to extract the content of a `camera/rgb` ROS topic and
treats the image as a matrix of pixels.

Notes:
-----
- Be sure that you loaded the Gazebo environment of tracking running the command:

```
$ roslaunch trackingbot tracking.launch
```

- Turtlebot must subscribe to `camera/rgb/image_raw` topic to access images from camera
- Turtlebot must publish to cmd_vel_mux/input/teleop`  topic to move across the field

"""
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Turtlebot:
  def __init__(self):
    """ interface between ros and opencv """
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)

    # Subscribe to ROS topics
     self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub= rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    self.twist = Twist()


  def image_callback(self, msg):
    """
    Callback of the ROS camera subscription
    
    `image_callback` should contain the code to:
      - identify the line in the image
      - calculate the center of the line (you may use the `cv2.moments()` function)
      - keep the line in the center of the image as the turtlebot moves
      - e.g. calculate the error between the center of the line and the
             center of the image. Publish the angular velocity as a 
             percentage of the error
    """
    # reads the image from camera topic and transform to OpenCV format
    image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    h, w, d = image.shape

    # <Your code here>

    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('turtlebot_tracker')
turtlebot = Turtlebot()
rospy.spin()
