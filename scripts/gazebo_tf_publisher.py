#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import tf

from gazebo_msgs.msg import ModelStates

class Tf_publisher():
    def __init__(self):
        self.model_name = rospy.get_param("~model_name")
        self.base_frame_id = rospy.get_param("~base_frame_id")
        self.global_frame_id = rospy.get_param("~global_frame_id")
        self.model_index = None
        self.first_sub = True
        self.model_pose = None

        # topic name
        self.gazebo_model_topic_name = "/gazebo/model_states"

        # subscriber
        self.gazebo_model_sub = rospy.Subscriber(self.gazebo_model_topic_name, ModelStates, self.gazebo_model_callback)

        # tf broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

    def gazebo_model_callback(self, _gazebo_model):
        if self.first_sub == True:
            self.model_index = _gazebo_model.name.index(self.model_name)
            self.first_sub = False
        self.model_pose = _gazebo_model.pose[self.model_index]

    def boroadcast_tf(self):
        x = self.model_pose.position.x
        y = self.model_pose.position.y
        z = self.model_pose.position.z
        qx = self.model_pose.orientation.x
        qy = self.model_pose.orientation.y
        qz = self.model_pose.orientation.z
        qw = self.model_pose.orientation.w
        #rospy.loginfo("{} : ({}, {})".format(self.model_name, x, y))
        self.tf_broadcaster.sendTransform((x, y, z), (qx, qy, qz,  qw), rospy.Time.now(), self.base_frame_id, self.global_frame_id)
        #rospy.loginfo("{} tf publish".format(self.model_name))

def main():
    rospy.init_node("gazebo_tf_publisher")
    tf_pub = Tf_publisher()
    rate = rospy.Rate(50)

    while tf_pub.model_pose is None:
        rate.sleep()

    while not rospy.is_shutdown():
        tf_pub.boroadcast_tf()
        rate.sleep()


if __name__ == '__main__':
    main()