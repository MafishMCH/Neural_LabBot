
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import os
from os import listdir
from os.path import isfile, join

import cv2
from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf
import tensorflow.keras
from keras.models import model_from_json


global image_data
img_width = 128
img_height = 96

# Publish robot speed in speed topic
speed_topic_name = "/cmd_joy"
pub = rospy.Publisher(speed_topic_name, Twist, queue_size=10) # Check type

# Load trained Neural Network
def load_model(model_json, model_weights):
    global model
    json_file = open(model_json, 'r')
    model_json = json_file.read()
    model = model_from_json(model_json, custom_objects={"GlorotUniform": tf.keras.initializers.glorot_uniform})
    model.load_weights(model_weights)
    #model.make_predict_funciton()

load_model('model.json', "model.h5")

def callback(data):
    global image_data
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # Store image data
    image_data = data


def robot_node():
    # Init node
    node_name = 'robot_moving_node'
    rospy.init_node(node_name, anonymous=True)

    # Subscribe to robot camera image topic
    camera_topic_name = "/camera/rgb/image_color"
    rospy.Subscriber(camera_topic_name, Image, callback)  # Change Image to image Type

    rate = rospy.Rate(10)  # 10hz
    rospy.sleep(1)
    bridge = CvBridge()
    while not rospy.is_shutdown():
	print("loop")
        # Convert image data to opencv image
        image = bridge.imgmsg_to_cv2(image_data, desired_encoding="passthrough")
        # Convert image to Grayscale
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Resize image
        image = cv2.resize(image, (img_width, img_height))
        # Transform image into Neural Network input
        nn_input = image.reshape(1, img_height, img_width, 1)
        # Predict speed using the Neural Network
        speed = model.predict(nn_input)
        print("works")
        # Publish speed
        vel_msg = Twist()
        vel_msg.linear.x = -0.4
        vel_msg.angular.z = 0.3 * speed
        pub.publish(vel_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        robot_node()
    except rospy.ROSInterruptException:
        pass

