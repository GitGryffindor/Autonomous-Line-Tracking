#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# lane_filter.py - Color space masking helper (ported to Python 3, logic unchanged)
#

import numpy as np
import cv2


class LaneFilter:
    def __init__(self):
        pass

    def red_color_mask(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        low_red = np.array([(0, 100, 100), (160, 100, 100)], dtype=np.uint8)
        upp_red = np.array([(19, 255, 255), (179, 255, 255)], dtype=np.uint8)
        mask1 = cv2.inRange(hsv, low_red[0], upp_red[0])
        mask2 = cv2.inRange(hsv, low_red[1], upp_red[1])
        mask = mask1 + mask2
        red_binary = np.dstack((mask, mask, mask))
        return red_binary

    def lab_color_mask(self, image):
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        a_channel_lab = lab[:, :, 1]
        a_thresh = (140, 255)
        a_binary = np.zeros_like(a_channel_lab)
        a_binary[(a_channel_lab >= a_thresh[0]) & (a_channel_lab <= a_thresh[1])] = 1
        a_binary = np.dstack((a_binary, a_binary, a_binary)) * 255
        return a_binary

    def luv_color_mask(self, image):
        luv = cv2.cvtColor(image, cv2.COLOR_BGR2LUV)
        u_channel_luv = luv[:, :, 1]
        u_thresh = (112, 255)
        u_binary = np.zeros_like(u_channel_luv)
        u_binary[(u_channel_luv >= u_thresh[0]) & (u_channel_luv <= u_thresh[1])] = 1
        u_binary = np.dstack((u_binary, u_binary, u_binary)) * 255
        return u_binary

    def ycrcb_color_mask(self, image):
        ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
        cr_channel_ycrcb = ycrcb[:, :, 1]
        cr_thresh = (140, 255)
        cr_binary = np.zeros_like(cr_channel_ycrcb)
        cr_binary[(cr_channel_ycrcb >= cr_thresh[0]) & (cr_channel_ycrcb <= cr_thresh[1])] = 1
        cr_binary = np.dstack((cr_binary, cr_binary, cr_binary)) * 255
        return cr_binary

    def processed_img(self, image):
        # Original code uses the Lab 'a' channel binary as the combined output
        red_binary = self.red_color_mask(image)    # noqa: F841
        a_binary = self.lab_color_mask(image)
        u_binary = self.luv_color_mask(image)       # noqa: F841
        cr_binary = self.ycrcb_color_mask(image)    # noqa: F841
        combined_binary = a_binary
        return combined_binary
