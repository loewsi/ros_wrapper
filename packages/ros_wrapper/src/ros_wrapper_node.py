#!/usr/bin/env python3

# import external libraries
import rospy
import os
import gym_duckietown
from gym_duckietown.simulator import Simulator
import cv2
import numpy as np



# import DTROS-related classes
from duckietown.dtros import \
    DTROS, \
    NodeType, \
    TopicType, \
    DTParam, \
    ParamType

# import messages and services
from std_msgs.msg import Float32
from duckietown_msgs.msg import \
    WheelsCmdStamped

from sensor_msgs.msg import CompressedImage, CameraInfo

class RosWrapperNode(DTROS):
    def __init__(self, node_name):
        super(RosWrapperNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.VISUALIZATION
        )

        self.action_l = 0.
        self.action_r = 0.



        self.sub_wheels_cmd = rospy.Subscriber(
            "/fakebot/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            self.wheels_cmd_cb,
            queue_size=1
        )

        self.pub_img = rospy.Publisher(
            "/fakebot/image/compressed",
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.DRIVER,
        )



    def wheels_cmd_cb(self, msg):
        self.action_l = msg.vel_left
        self.action_r = msg.vel_right



    def publish_img(self, obs):
        img_msg = CompressedImage()
        time = rospy.get_rostime()

        img_msg.header.stamp.secs = time.secs
        img_msg.header.stamp.nsecs = time.nsecs

        img_msg.format = "jpeg"
        image = cv2.cvtColor(np.ascontiguousarray(obs), cv2.COLOR_BGR2RGB)
        img_msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()

        self.pub_img.publish(img_msg)



if __name__ == '__main__':
    wrapper = RosWrapperNode(node_name='ros_wrapper_node')
    env = Simulator(
        seed=123, # random seed
        map_name="loop_empty",
        max_steps=500001, # we don't want the gym to reset itself
        domain_rand=0,
        camera_width=640,
        camera_height=480,
        accept_start_angle_deg=4, # start close to straight
        full_transparency=True,
        distortion=True,
    )
    obs = env.reset()
    wrapper.publish_img(obs)

    while not rospy.is_shutdown():
        action = [wrapper.action_l, wrapper.action_r]
        obs, reward, done, _ = env.step(action)

        if done:
            obs = env.reset()

        wrapper.publish_img(obs)
        rospy.Rate(15).sleep()



