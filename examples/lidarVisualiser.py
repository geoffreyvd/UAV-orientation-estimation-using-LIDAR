#!/usr/bin/env python3

# Arbitrary display params
DISPLAY_CANVAS_SIZE_PIXELS  = 980
DISPLAY_CANVAS_COLOR        = 'black'
DISPLAY_SCAN_LINE_COLOR     = 'yellow'
DISPLAY_SCAN_LINE_COLOR_LARGEST_DISTANCE     = 'red'
DISPLAY_SCAN_LINE_COLOR_EXTRACTED_LINES     = 'blue'
DISPLAY_SCAN_LINE_COLOR_PERPENDICULAR     = 'green'

# URG-04LX specs
URG_MAX_SCAN_DIST_MM        = 2000
URG_DETECTION_DEG           = 240
URG_SCAN_SIZE               = 682
from sys import exit, version
from math import sin, cos, radians, atan, atan2, pi, fabs, sqrt

if version[0] == '3':
    import tkinter as tk
else:
    import Tkinter as tk

class lidarVisualiser(tk.Frame):
    ""

    def __init__(self):

        # Create the frame        
        tk.Frame.__init__(self, borderwidth = 4, relief = 'sunken')
        self.master.geometry(str(DISPLAY_CANVAS_SIZE_PIXELS)+ "x" + str(DISPLAY_CANVAS_SIZE_PIXELS))
        self.master.title('Hokuyo URG04LX  [ESC to quit]')
        self.grid()
        self.master.rowconfigure(0, weight = 1)
        self.master.columnconfigure(0, weight = 1)
        self.grid(sticky = tk.W+tk.E+tk.N+tk.S)
        self.background = DISPLAY_CANVAS_COLOR

        self.half_canvas_pix = DISPLAY_CANVAS_SIZE_PIXELS / 2
        self.renderScale = self.half_canvas_pix / float(URG_MAX_SCAN_DIST_MM)

        #pre calculate some value for plotting and mathss
        scan_angle_rad = [radians(-URG_DETECTION_DEG/2 + (float(k)/URG_SCAN_SIZE) * \
                                   URG_DETECTION_DEG) for k in range(URG_SCAN_SIZE)]
        self.scanRadians = scan_angle_rad
        self.scanAngles = [scan_angle_rad[k]*180/pi for k in range(URG_SCAN_SIZE)]

        self.cos = [-cos(angle) * self.renderScale for angle in scan_angle_rad]
        self.sin = [ sin(angle) * self.renderScale for angle in scan_angle_rad]
        self.cosRaw = [ cos(angle) for angle in scan_angle_rad]
        self.sinRaw = [ sin(angle) for angle in scan_angle_rad]    

        self.showCount = 0

        # Add a canvas for drawing
        self.canvas =  tk.Canvas(self, \
            width = DISPLAY_CANVAS_SIZE_PIXELS, \
            height = DISPLAY_CANVAS_SIZE_PIXELS,\
            background = DISPLAY_CANVAS_COLOR)
        self.canvas.grid(row = 0, column = 0,\
                    rowspan = 1, columnspan = 1,\
                    sticky = tk.W+tk.E+tk.N+tk.S)

        self.perpendicularLineCount = 0
        self.extractedLinesCountDebug = 0
        # No scanlines initially                             
        self.linesLidar = []

        # Add scan lines to canvas, to be modified later
        self.linesLidar = [self.canvas.create_line(\
                         self.half_canvas_pix, \
                         self.half_canvas_pix, \
                         self.half_canvas_pix + self.sin[k] * URG_MAX_SCAN_DIST_MM,\
                         self.half_canvas_pix + self.cos[k] * URG_MAX_SCAN_DIST_MM)
                         for k in range(URG_SCAN_SIZE)]
        self.linesExtracted = [self.canvas.create_line(\
                         0, \
                         0, \
                         self.sin[k] * 10,\
                         self.cos[k] * 10)
                         for k in range(URG_SCAN_SIZE)]
        self.perpendicularLines = [self.canvas.create_line(\
                         0, \
                         0, \
                         self.sin[k] * 10,\
                         self.cos[k] * 10)
                         for k in range(URG_SCAN_SIZE)]

        [self.canvas.itemconfig(line, fill=DISPLAY_SCAN_LINE_COLOR) for line in self.linesLidar]
        print("origin x and y: {}, renderScale: {}".format(self.half_canvas_pix, self.renderScale))

    def plotScanDataPointsAsLines(self, scandata):
        # Modify the yellow displayed lines according to the current scan
        [self.canvas.coords(self.linesLidar[k], 
                            self.half_canvas_pix, \
                            self.half_canvas_pix, \
                            self.half_canvas_pix + self.sin[k] * scandata[k],\
                            self.half_canvas_pix + self.cos[k] * scandata[k]) \
         for k in range(len(scandata))]
        self.showCount += 1   

    def plotWalls(self, walls):        
        for idx, w in enumerate(walls):
            print("idx: {}, point 1 idx: {}, point 2 idx: {}".format(idx, w.index1, w.index2))
            self.canvas.coords(self.linesExtracted[idx], w.x1, w.y1, w.x2, w.y2)
            widthWall = 1 + 10/(idx +1)
            self.canvas.itemconfig(self.linesExtracted[idx], fill=DISPLAY_SCAN_LINE_COLOR_EXTRACTED_LINES, width=widthWall) 
    
    def plotPerpendicularLines(self, perpendicularDistanceRaw, perpendicularRadian):
        perpendicularDistance = perpendicularDistanceRaw * self.renderScale
        perpendicularLineEndX = self.half_canvas_pix+ perpendicularDistance * sin(perpendicularRadian)
        perpendicularLineEndY = self.half_canvas_pix+ perpendicularDistance * -cos(perpendicularRadian)
        self.perpendicularLineCount += 1
        [self.canvas.coords(self.perpendicularLines[self.perpendicularLineCount], 
                        perpendicularLineEndX, \
                        perpendicularLineEndY, \
                        self.half_canvas_pix,\
                        self.half_canvas_pix)]    
        [self.canvas.itemconfig(self.perpendicularLines[self.perpendicularLineCount], fill=DISPLAY_SCAN_LINE_COLOR_PERPENDICULAR)]

    def plotSplitLine(self, firstPointX, firstPointY, lastPointX, lastPointY):
        self.extractedLinesCountDebug+=1
        [self.canvas.coords(self.linesExtracted[self.extractedLinesCountDebug], 
                        firstPointX, \
                        firstPointY, \
                        lastPointX,\
                        lastPointY)]    
        [self.canvas.itemconfig(self.linesExtracted[self.extractedLinesCountDebug], fill=DISPLAY_SCAN_LINE_COLOR_EXTRACTED_LINES)]

    def plotLargestDistance(self, indexLargestDistance):
        [self.canvas.itemconfig(self.linesLidar[indexLargestDistance], fill=DISPLAY_SCAN_LINE_COLOR_LARGEST_DISTANCE)]
        
    def updateGUI(self):
        self.extractedLinesCountDebug = 0 #root of evil - side functionality
        self.update()
    
    def getShowCount(self):
        return self.showCount