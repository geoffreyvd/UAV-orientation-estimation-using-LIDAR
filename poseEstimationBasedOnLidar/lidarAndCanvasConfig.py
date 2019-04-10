#!/usr/bin/env python3

DISPLAY_CANVAS_SIZE_PIXELS  = 980
# URG-04LX specs
URG_MAX_SCAN_DIST_MM        = 2000
URG_DETECTION_DEG           = 240
URG_SCAN_SIZE               = 682

from math import sin, cos, radians, atan, atan2, pi, fabs, sqrt

class lidarAndCanvasConfig():
    
    def __init__(self):        

        self.half_canvas_pix = DISPLAY_CANVAS_SIZE_PIXELS / 2
        self.renderScale = self.half_canvas_pix / float(URG_MAX_SCAN_DIST_MM)

        # Pre-compute some values useful for plotting        
        scan_angle_rad = [radians(-URG_DETECTION_DEG/2 + (float(k)/URG_SCAN_SIZE) * \
                                   URG_DETECTION_DEG) for k in range(URG_SCAN_SIZE)]
        self.scanRadians = scan_angle_rad
        self.scanAngles = [scan_angle_rad[k]*180/pi for k in range(URG_SCAN_SIZE)]

        self.cos = [-cos(angle) * self.renderScale for angle in scan_angle_rad]
        self.sin = [ sin(angle) * self.renderScale for angle in scan_angle_rad]
        self.cosRaw = [ cos(angle) for angle in scan_angle_rad]
        self.sinRaw = [ sin(angle) for angle in scan_angle_rad]        