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


class Lrs2Grid():
    def __init__(self):
        # get parameters
        self.save_path = rospy.get_param("~save_path")
        lrs_topic = rospy.get_param("~lrs_topic")
        self.grid_size = rospy.get_param("~grid_size")
        self.map_size = [rospy.get_param("~x_min"), rospy.get_param("~x_max"), rospy.get_param("~y_min"), rospy.get_param("~y_max")]

        # initialize
        self.laser_pjojection = lg.LaserProjection()
        self.points_num = None
        self.lrs_frame = None
        self.sub_lrs = rospy.Subscriber(lrs_topic, LaserScan, self.lrs_handler)
        self.tf_listener = tf.TransformListener()
        self.save_file_index = 0
        self.processing_flag = False
        self.rate = rospy.Rate(10)

    def lrs_handler(self, _msg):
        if not self.processing_flag:
            self.processing_flag = True
            if self.lrs_frame is None:
                self.lrs_frame = _msg.header.frame_id
            pc2_msg = self.laser_pjojection.projectLaser(_msg)
            points_arr = self.generate_arr(pc2_msg)
            points_arr_tran = self.transform_points(points_arr)
            self.generate_grid(points_arr_tran, self.grid_size, self.map_size)

            # wait
            self.rate.sleep()

            self.processing_flag = False

    def generate_arr(self, _pc2_msg):
        point_generator = pc2.read_points(_pc2_msg)
        points = []
        # start_time = time.time()
        for point in point_generator:
            if not math.isnan(point[2]):
                points.append([point[0], point[1]])  # (x, y)
        self.points_num = len(points)
        points_arr = np.array(points)
        # print(time.time() - start_time)
        return points_arr

    def transform_points(self, _points_arr):
        [x, y, th] = self.get_tf()

        print("theta : {}".format(th / math.pi * 180))


        # homogeneous coordinate system
        points_arr_homo = np.append(_points_arr.T, np.ones((1, self.points_num), dtype=np.float32), axis=0)

        # translation matrix
        th = math.pi /2 - th
        translation_mat = np.matrix([[math.cos(th), -math.sin(th), x], [math.sin(th), math.cos(th), y], [0, 0, 1]], dtype=np.float32)

        print(translation_mat)

        # translation
        points_arr_homo_tran = translation_mat * points_arr_homo

        points_arr_tran = np.delete(points_arr_homo_tran, obj=2, axis=0).T

        return points_arr_tran

    def generate_grid(self, _arr, _grid_size, _map_size):
        [x_min, x_max, y_min, y_max] = _map_size  # [m]
        grid_x_min = round(x_min / _grid_size)
        grid_x_max = round(x_max / _grid_size)
        grid_y_min = round(y_min / _grid_size)
        grid_y_max = round(y_max / _grid_size)

        grid_x_width = grid_x_max - grid_x_min + 1
        grid_y_width = grid_y_max - grid_y_min + 1

        grid_arr = np.zeros((int(grid_x_width), int(grid_y_width)), dtype=np.int)
        grid_arr = np.full_like(grid_arr, 255)

        arr_round = np.round(_arr / _grid_size)

        for point_i in range(self.points_num):
            x_point = arr_round[point_i][0, 0]
            y_point = arr_round[point_i][0, 1]
            grid_arr[int(x_point - grid_x_min), int(y_point - grid_y_min)] = 0

        cv2.imwrite(self.save_path + "/" + str(self.save_file_index) + ".jpg", grid_arr)
        print("save_file_index : {}".format(self.save_file_index))
        self.save_file_index += 1

    def get_tf(self):
        print("get_tf")
        while not rospy.is_shutdown():
            try:
                (translation, rotation) = self.tf_listener.lookupTransform("/map", self.lrs_frame, rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        [x, y, z] = translation
        [roll, pitch, yaw] = tf.transformations.euler_from_quaternion(rotation)

        return x, y, yaw


def main():
    rospy.init_node("lrs_to_grid_converter")

    print("===== LRS Grid Convertor ======================")
    lrs2grid = Lrs2Grid()

    rospy.spin()


if __name__ == '__main__':
    main()
