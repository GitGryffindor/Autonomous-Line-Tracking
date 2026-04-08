#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# curve_fit.py - Sliding window + polynomial lane fitting (ported to Python 3)
# Key Python 3 fix: np.int() is deprecated -> replaced with int()
#

import numpy as np
import cv2


class CurveFit:
    def __init__(self, number_of_windows, margin, minimum_pixels):
        self.nwindows = number_of_windows
        self.margin = margin
        self.minpix = minimum_pixels
        self.output_img = None

    def method_two(self, binary_warped_image, lane_fit):
        nonzerox = np.array(binary_warped_image.nonzero()[1])
        nonzeroy = np.array(binary_warped_image.nonzero()[0])
        self.output_img = binary_warped_image.copy()

        lane_coor = (
            (nonzerox > (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] - self.margin)) &
            (nonzerox < (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] + self.margin)))
        x = nonzerox[lane_coor]
        y = nonzeroy[lane_coor]
        self.output_img[nonzeroy[lane_coor], nonzerox[lane_coor]] = [255, 200, 200]

        try:
            lane_fit = np.polyfit(y, x, 2)
            ploty = np.linspace(0, binary_warped_image.shape[0] - 1, binary_warped_image.shape[0])
            lane_fitx = lane_fit[0] * (ploty ** 2) + lane_fit[1] * ploty + lane_fit[2]
            pts = np.array([np.transpose(np.vstack([lane_fitx, ploty]))])
            cv2.polylines(self.output_img, np.int32([pts]), False, (255, 100, 100), 15)
        except Exception:
            lane_fit = None

        return lane_fit

    def sliding_windows(self, binary_warped_image):
        nonzerox = np.array(binary_warped_image.nonzero()[1])
        nonzeroy = np.array(binary_warped_image.nonzero()[0])

        histogram = np.sum(binary_warped_image[binary_warped_image.shape[0] // 2:, :], axis=0)
        x_base = int(np.mean(np.argmax(histogram, axis=0)))
        x_current = x_base
        # Python 3 fix: np.int() -> int()
        window_height = int(binary_warped_image.shape[0] / self.nwindows)
        lane_coor = []

        for w in range(self.nwindows):
            wy_low = binary_warped_image.shape[0] - (w + 1) * window_height
            wy_high = binary_warped_image.shape[0] - w * window_height
            wx_low = x_current - self.margin
            wx_high = x_current + self.margin

            found_coor = (
                (nonzeroy >= wy_low) & (nonzeroy < wy_high) &
                (nonzerox >= wx_low) & (nonzerox < wx_high)
            ).nonzero()[0]
            lane_coor.append(found_coor)

            if len(found_coor) > self.minpix:
                x_current = int(np.mean(nonzerox[found_coor]))

        lane_coor = np.concatenate(lane_coor)
        x = nonzerox[lane_coor]
        y = nonzeroy[lane_coor]

        try:
            lane_fit = np.polyfit(y, x, 2)
        except Exception:
            lane_fit = None

        return lane_fit

    def plot(self):
        return self.output_img
