#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
import time
import cv2
import tf
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import laser_geometry.laser_geometry as lg
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointField
from std_msgs.msg import Header


class Lrs2Pc2():
    def __init__(self):
        # get parameters
        lrs_topic = rospy.get_param("~lrs_topic")
        lrs_pc2_topic = rospy.get_param("~lrs_pc2_topic")

        # initialize
        self.laser_pjojection = lg.LaserProjection()
        self.points_num = None
        self.lrs_frame = None
        self.sub_lrs = rospy.Subscriber(lrs_topic, LaserScan, self.lrs_handler)
        self.pub_lrs_pc2 = rospy.Publisher(lrs_pc2_topic, PointCloud2, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.processing_flag = False

    def lrs_handler(self, _msg):
        if not self.processing_flag:
            self.processing_flag = True
            if self.lrs_frame is None:
                self.lrs_frame = _msg.header.frame_id
            pc2_msg = self.laser_pjojection.projectLaser(_msg)
            self.pub_lrs_pc2.publish(pc2_msg)
            self.processing_flag = False

    def generate_pc2(self, _points):
        header = Header(frame_id=self.lrs_frame)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        pc2_msg = pc2.create_cloud(header, fields, _points)
        return pc2_msg

def main():
    rospy.init_node("lrs_to_grid_converter")

    print("===== LRS to PointCloud2 Convertor ======================")
    lrs2Pc2 = Lrs2Pc2()

    rospy.spin()


if __name__ == '__main__':
    main()
