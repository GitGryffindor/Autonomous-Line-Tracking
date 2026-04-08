#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# birds_eye_view.py - Perspective warp helper (ported to Python 3, logic unchanged)
#

import numpy as np
import cv2


class BirdsEyeView:
    def __init__(self, source_points, destination_points):
        self.src = source_points
        self.dst = destination_points
        self.warp_matrix = cv2.getPerspectiveTransform(self.src, self.dst)
        self.inv_warp_matrix = cv2.getPerspectiveTransform(self.dst, self.src)

    def sky_view(self, robot_view_image):
        return cv2.warpPerspective(
            robot_view_image, self.warp_matrix,
            (robot_view_image.shape[1], robot_view_image.shape[0]))

    def project_lanes(self, robot_view_image,
                      skyview_image, lane_fit, lane_color=(255, 0, 255), lane_thickness=40):
        warp_mask = np.zeros_like(skyview_image)
        ploty = np.linspace(0, skyview_image.shape[0] - 1, skyview_image.shape[0])
        lane_fitx = lane_fit[0] * (ploty ** 2) + lane_fit[1] * ploty + lane_fit[2]
        pts = np.array([np.transpose(np.vstack([lane_fitx, ploty]))])
        cv2.polylines(warp_mask, np.int32([pts]), False, lane_color, lane_thickness)
        warp_lane = cv2.warpPerspective(
            warp_mask,
            self.inv_warp_matrix,
            (robot_view_image.shape[1], robot_view_image.shape[0]))
        result = cv2.addWeighted(robot_view_image, 1, warp_lane, 0.8, 0)
        return result
