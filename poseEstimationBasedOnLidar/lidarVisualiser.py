#!/usr/bin/env python3

# Arbitrary display params
DISPLAY_CANVAS_COLOR        = 'black'
DISPLAY_SCAN_LINE_COLOR     = 'yellow'
DISPLAY_SCAN_LINE_COLOR_LARGEST_DISTANCE     = 'red'
DISPLAY_SCAN_LINE_COLOR_EXTRACTED_LINES     = 'blue'
DISPLAY_SCAN_LINE_COLOR_PERPENDICULAR     = 'green'

from sys import exit, version
from math import sin, cos, radians, atan, atan2, pi, fabs, sqrt
from lidarAndCanvasConfig import DISPLAY_CANVAS_SIZE_PIXELS, URG_MAX_SCAN_DIST_MM, URG_DETECTION_DEG, URG_SCAN_SIZE

if version[0] == '3':
    import tkinter as tk
else:
    import Tkinter as tk

class lidarVisualiser(tk.Frame):
    ""

    def __init__(self, config):
        self.config = config

        # Create the frame        
        tk.Frame.__init__(self, borderwidth = 4, relief = 'sunken')
        self.master.geometry(str(DISPLAY_CANVAS_SIZE_PIXELS)+ "x" + str(DISPLAY_CANVAS_SIZE_PIXELS))
        self.master.title('Hokuyo URG04LX  [ESC to quit]')
        self.grid()
        self.master.rowconfigure(0, weight = 1)
        self.master.columnconfigure(0, weight = 1)
        self.grid(sticky = tk.W+tk.E+tk.N+tk.S)
        self.background = DISPLAY_CANVAS_COLOR 

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
                         self.config.half_canvas_pix, \
                         self.config.half_canvas_pix, \
                         self.config.half_canvas_pix + self.config.sin[k] * URG_MAX_SCAN_DIST_MM,\
                         self.config.half_canvas_pix + self.config.cos[k] * URG_MAX_SCAN_DIST_MM)
                         for k in range(URG_SCAN_SIZE)]
        self.linesExtracted = [self.canvas.create_line(\
                         0, \
                         0, \
                         self.config.sin[k] * 10,\
                         self.config.cos[k] * 10)
                         for k in range(URG_SCAN_SIZE)]
        self.perpendicularLines = [self.canvas.create_line(\
                         0, \
                         0, \
                         self.config.sin[k] * 10,\
                         self.config.cos[k] * 10)
                         for k in range(URG_SCAN_SIZE)]

        [self.canvas.itemconfig(line, fill=DISPLAY_SCAN_LINE_COLOR) for line in self.linesLidar]
        print("origin x and y: {}, renderScale: {}".format(self.config.half_canvas_pix, self.config.renderScale))

    def plotScanDataPointsAsLines(self, scandata):
        # Modify the yellow displayed lines according to the current scan
        [self.canvas.coords(self.linesLidar[k], 
                            self.config.half_canvas_pix, \
                            self.config.half_canvas_pix, \
                            self.config.half_canvas_pix + self.config.sin[k] * scandata[k],\
                            self.config.half_canvas_pix + self.config.cos[k] * scandata[k]) \
         for k in range(len(scandata))]
        self.showCount += 1   

    def plotWalls(self, walls):        
        for idx, w in enumerate(walls):
            self.canvas.coords(self.linesExtracted[idx], w.x1, w.y1, w.x2, w.y2)
            widthWall = 1 + 10/(idx +1)
            self.canvas.itemconfig(self.linesExtracted[idx], fill=DISPLAY_SCAN_LINE_COLOR_EXTRACTED_LINES, width=widthWall) 
    
    def plotPerpendicularLines(self, perpendicularDistanceRaw, perpendicularRadian):
        perpendicularDistance = perpendicularDistanceRaw * self.config.renderScale
        perpendicularLineEndX = self.config.half_canvas_pix+ perpendicularDistance * sin(perpendicularRadian)
        perpendicularLineEndY = self.config.half_canvas_pix+ perpendicularDistance * -cos(perpendicularRadian)
        self.perpendicularLineCount += 1
        [self.canvas.coords(self.perpendicularLines[self.perpendicularLineCount], 
                        perpendicularLineEndX, \
                        perpendicularLineEndY, \
                        self.config.half_canvas_pix,\
                        self.config.half_canvas_pix)]    
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
    
    def applyScaleToPoint(self, x1Raw, y1Raw):
        pointX = self.config.half_canvas_pix + x1Raw * self.config.renderScale
        pointY = self.config.half_canvas_pix + -y1Raw * self.config.renderScale
        return pointX, pointY

    def updateGUI(self):
        self.extractedLinesCountDebug = 0 #root of evil - side functionality
        self.update()
    
    def getShowCount(self):
        return self.showCount