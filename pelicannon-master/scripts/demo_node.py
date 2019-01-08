#!/usr/bin/env python

import datetime
from cv_bridge import CvBridge
from threading import Lock

import cv2
import numpy as np
import rospy

from jetson_tensorrt.msg import ClassifiedRegionsOfInterest
from jetson_tensorrt.msg import Classifications
from sensor_msgs.msg import Image

class DemoNode(object):
    def __init__(self):
        self._cv_br = CvBridge()

        self._ros_init()

        self._roi_lock = Lock()
        self._latest_rois = []

        self._depth_lock = Lock()
        self._latest_depth = None

    def _ros_init(self):
        rospy.init_node('debug')

        rospy.Subscriber('/detector/detections', ClassifiedRegionsOfInterest, self._detect_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('image_topic', '/camera/color/image_raw'), Image, self._camera_callback, queue_size=2)
        rospy.Subscriber(rospy.get_param('depth_topic', '/camera/depth/image_rect_raw'), Image, self._depth_callback, queue_size=2)
        self._publisher = rospy.Publisher('demo', Image, queue_size=2)

    def _detect_callback(self, regions):

        with self._roi_lock:
            self._latest_rois = regions.regions

    def _depth_callback(self, depth):
        with self._depth_lock:
            self._latest_depth = self._cv_br.imgmsg_to_cv2(depth)

    def _camera_callback(self, image):

        frame = self._cv_br.imgmsg_to_cv2(image)

        rois = []
        with self._roi_lock:
            rois = self._latest_rois

        depth = None
        with self._depth_lock:
            depth = self._latest_depth

        for roi in rois:
            cv2.rectangle(frame, (roi.x, roi.y),
                          (roi.x + roi.w, roi.y + roi.h), (0, 255, 0), 10)

            if depth is not None:
                y_scale = depth.shape[0] / frame.shape[0]
                x_scale = depth.shape[1] / frame.shape[1]

                d_x = int(roi.x * x_scale)
                d_y = int(roi.y * y_scale)
                d_w = int(roi.w * x_scale)
                d_h = int(roi.h * y_scale)

                sub_depth = depth[d_y:d_y+d_h,d_x:d_x+d_w]

                median_distance = np.median(sub_depth) / 1000.0

                cv2.putText(frame, "%.1f m" % median_distance, (max(0, roi.x), max(0, roi.y - 20)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        self._publisher.publish(self._cv_br.cv2_to_imgmsg(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))


if __name__ == "__main__":
    node = DemoNode()
    rospy.spin()
