#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import pygame.camera
import pygame.image
import sys
import time
import sensor_msgs.msg
try:
    from cStringIO import StringIO, InputType, OutputType
    from StringIO import StringIO as pyStringIO

    def _checkIsStringIO(obj):
        return isinstance(obj, (InputType, OutputType, pyStringIO))
except ImportError:
    from StringIO import StringIO

    def _checkIsStringIO(obj):
        return isinstance(obj, StringIO)

class ImageConverter(object):
    """
    Convert images/compressedimages to and from ROS
    """

    _ENCODINGMAP_PY_TO_ROS = {'L': 'mono8', 'RGB': 'rgb8',
                              'RGBA': 'rgba8', 'YCbCr': 'yuv422'}
    _ENCODINGMAP_ROS_TO_PY = {'mono8': 'L', 'rgb8': 'RGB',
                              'rgba8': 'RGBA', 'yuv422': 'YCbCr'}
    _PIL_MODE_CHANNELS = {'L': 1, 'RGB': 3, 'RGBA': 4, 'YCbCr': 3}

    @staticmethod
    def to_ros(img):
        """
        Convert a PIL/pygame image to a ROS compatible message (sensor_msgs.Image).
        """

        # Everything ok, convert PIL.Image to ROS and return it
        if img.mode == 'P':
            img = img.convert('RGB')

        rosimage = sensor_msgs.msg.Image()
        rosimage.encoding = ImageConverter._ENCODINGMAP_PY_TO_ROS[img.mode]
        (rosimage.width, rosimage.height) = img.size
        rosimage.step = (ImageConverter._PIL_MODE_CHANNELS[img.mode]
                         * rosimage.width)
        rosimage.data = img.tostring()
        return rosimage

    @classmethod
    def from_ros(cls, rosMsg):
        """
        Converts a ROS sensor_msgs.Image or sensor_msgs.CompressedImage to a pygame Surface
        :param rosMsg: The message to convert
        :return: an alpha-converted pygame Surface
        """
        pyimg = None
        if isinstance(rosMsg, sensor_msgs.msg.Image):
            pyimg = pygame.image.fromstring(rosMsg.data, (rosMsg.width, rosMsg.height),
                                            cls._ENCODINGMAP_ROS_TO_PY[rosMsg.encoding])
        elif isinstance(rosMsg, sensor_msgs.msg.CompressedImage):
            pyimg = pygame.image.load(StringIO(rosMsg.data))

        if not pyimg:
            raise TypeError('rosMsg is not an Image or CompressedImage!')

        return pyimg.convert_alpha()

conv = ImageConverter()
pygame.camera.init()
cameras = pygame.camera.list_cameras()

print "Using camera %s ..." % cameras[0]

webcam = pygame.camera.Camera(cameras[0])

webcam.start()

# grab first frame
img = webcam.get_image()

WIDTH = img.get_width()
HEIGHT = img.get_height()

direction = "forward"
#screen = pygame.display.set_mode( ( WIDTH, HEIGHT ) )
#pygame.display.set_caption("pyGame Camera View")

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.angular.z)
	treshhold = 0.1
	if data.angular.z > treshhold:	direction = "right" 
	elif data.angular.z < -treshhold:	direction = "left"
	else:	direction = "forward"   
    
def callbackImg(data):
	rospy.loginfo(rospy.get_caller_id() + " Nowy obrazek")
	img = from_ros(conv, data)
	pygame.image.save(img, 'images/image%s.jpg' % ("-" + direction + "-"+ str(time.time())))
	
def listener():
	rospy.init_node("recorder")
	rospy.Subscriber("/cmd_vel", Twist, callback)
	rospy.Subscriber("/kinect/sd/image_color", sensor_msgs.CompressedImage, callbackImg)	#TODO
	rospy.spin()

listener()

