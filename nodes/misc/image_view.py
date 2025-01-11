#!/usr/bin/env python3

from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

import cv2
import numpy as np

from utils import ros2_utils

class IMAGE_VIEW(Node):
    def __init__(self, params):
        self.cv_win_nm = '_'.join(params["topic_name"].split("/")[1:])
        super().__init__("image_view_"+self.cv_win_nm)
        self.br = CvBridge()
        self.img = None
        qos_profile = ros2_utils.custom_qos_profile(params["queue_size"])
        if params["img_type"] == "compressed":
            self.sub_img = self.create_subscription(
                CompressedImage,
                params["topic_name"],
                self.compressed_callback,
                qos_profile
            )
        elif params["img_type"] == "raw":
            self.sub_img = self.create_subscription(
                Image,
                params["topic_name"],
                self.raw_callback,
                qos_profile
            )

    def raw_callback(self, img_msg):
        self.img = self.br.imgmsg_to_cv2(img_msg)
        if self.img is not None:
            # self.get_logger().info(f"{self.img[240:243, 720]}")
            # cv2.imwrite("/home/sitl-dvrk-sub/recon3dtest.png", self.img)
            cv2.imshow(self.cv_win_nm, self.img)
            cv2.waitKey(1)

    def compressed_callback(self, img_msg):
        self.img = self.br.compressed_imgmsg_to_cv2(img_msg)
        if self.img is not None:
            # ros2_utils.loginfo(self, f"{np.median(self.img)}")
            # self.get_logger().info(f"{self.img[150:153, 720]}")
            # cv2.imwrite("/home/sitl-dvrk-sub/recon3dtest.png", self.img)
            cv2.imshow(self.cv_win_nm, self.img)
            cv2.waitKey(1)
