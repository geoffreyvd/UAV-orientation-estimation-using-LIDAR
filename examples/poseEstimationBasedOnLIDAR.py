#!/usr/bin/env python3

'''
urgplot.py : A little Python class to display Lidar scans from the Hokuyo URG-04LX
             
Copyright (C) 2014 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

#NOTE, if this programs returns breezylidar connect error, then just retry 3 times
URG_DEVICE                  = '/dev/ttyACM0'

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

from breezylidar import URG04LX
from math import sin, cos, radians, atan, atan2, pi, fabs, sqrt
from time import time, sleep, ctime
from sys import exit, version
from splitAndMerge import extractedLine, calculatePerpendicularLine, linearRegression
# TODO, use numpy instead of python list

if version[0] == '3':
    import tkinter as tk
    import _thread as thread
else:
    import Tkinter as tk
    import thread

# Runs on its own thread
def grab_scan(obj):
    while True:
        scandata = obj.lidar.getScan()
        if scandata:
            obj.scandata = scandata
            obj.count += 1
            sleep(.01) # pause a tiny amount to allow following check to work
            if not obj.running:
                break
def multiplication(a, b):
    return a*b

class URGPlotter(tk.Frame):
    '''
    UGRPlotter extends tk.Frame to plot Lidar scans.
    '''
    
    def __init__(self):
        '''
        Takes no args.  Maybe we could specify colors, lidar params, etc.
        '''
        
        # Create the frame        
        tk.Frame.__init__(self, borderwidth = 4, relief = 'sunken')
        self.master.geometry(str(DISPLAY_CANVAS_SIZE_PIXELS)+ "x" + str(DISPLAY_CANVAS_SIZE_PIXELS))
        self.master.title('Hokuyo URG04LX  [ESC to quit]')
        self.grid()
        self.master.rowconfigure(0, weight = 1)
        self.master.columnconfigure(0, weight = 1)
        self.grid(sticky = tk.W+tk.E+tk.N+tk.S)
        self.background = DISPLAY_CANVAS_COLOR
        
        # Add a canvas for drawing
        self.canvas =  tk.Canvas(self, \
            width = DISPLAY_CANVAS_SIZE_PIXELS, \
            height = DISPLAY_CANVAS_SIZE_PIXELS,\
            background = DISPLAY_CANVAS_COLOR)
        self.canvas.grid(row = 0, column = 0,\
                    rowspan = 1, columnspan = 1,\
                    sticky = tk.W+tk.E+tk.N+tk.S)

        # Set up a key event for exit on ESC
        self.bind('<Key>', self._key)

        # This call gives the frame focus so that it receives input
        self.focus_set()

        # No scanlines initially                             
        self.linesLidar = []
        
        # Create a URG04LX object and connect to it
        self.lidar = URG04LX(URG_DEVICE)
        
        # No scan data to start
        self.scandata = []
        
        self.extractedLinesCountDebug = 0
        self.perpendicularLineCount = 0
        self.previousWalls = None

        #test
        self.listOfYaw = []
        self.listOfYawLR =[]
        
        # Pre-compute some values useful for plotting        
        scan_angle_rad = [radians(-URG_DETECTION_DEG/2 + (float(k)/URG_SCAN_SIZE) * \
                                   URG_DETECTION_DEG) for k in range(URG_SCAN_SIZE)]
        self.scanRadians = scan_angle_rad
        self.scanAngles = [scan_angle_rad[k]*180/pi for k in range(URG_SCAN_SIZE)]

        self.half_canvas_pix = DISPLAY_CANVAS_SIZE_PIXELS / 2
        self.renderScale = self.half_canvas_pix / float(URG_MAX_SCAN_DIST_MM)

        self.cos = [-cos(angle) * self.renderScale for angle in scan_angle_rad]
        self.sin = [ sin(angle) * self.renderScale for angle in scan_angle_rad]

        self.cosRaw = [ cos(angle) for angle in scan_angle_rad]
        self.sinRaw = [ sin(angle) for angle in scan_angle_rad]
        
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

        # Start a new thread and set a flag to let it know when we stop running
        thread.start_new_thread( grab_scan, (self,) )       
        self.running = True      
        
    def run(self):
        '''
        Call this when you're ready to run.
        '''
        
        # Record start time and initiate a count of scans for testing
        self.count = 0
        self.start_sec = time()
        self.showcount = 0
        
        # Start the recursive timer-task
        plotter._task() 
        
        # Start the GUI
        plotter.mainloop()
        
        
    def destroy(self):
        '''
        Called automagically when user clicks X to close window.
        '''  

        self._quit()

    def _quit(self):

        self.running = False
        elapsed_sec = time() - self.start_sec
        print('%d scans    in %f sec = %f scans/sec' % (self.count, elapsed_sec, self.count/elapsed_sec))
        print('%d displays in %f sec = %f displays/sec' % (self.showcount, elapsed_sec, self.showcount/elapsed_sec))
        
        del self.lidar

        exit(0)
        
    def _key(self, event):

        # Make sure the frame is receiving input!
        self.focus_force()
        if event.keysym == 'Escape':
            self._quit()

    def _task(self):
        # The only thing here that has to happen is setting the lengh of the line, by setting a new endpoint for the line 
        # x = cos * scanPointDistance, y = sin * scanPointDistance
        # Modify the yellow displayed lines according to the current scan
        scandata = self.scandata[:] #make copy of data, very important because of the parallel thread
        [self.canvas.coords(self.linesLidar[k], 
                            self.half_canvas_pix, \
                            self.half_canvas_pix, \
                            self.half_canvas_pix + self.sin[k] * scandata[k],\
                            self.half_canvas_pix + self.cos[k] * scandata[k]) \
         for k in range(len(scandata))]    
        if scandata:
            i = 0
            firstValidPoint = -1
            while i == 0:
                firstValidPoint += 1
                i = scandata[firstValidPoint]
            
            i = 0
            lastValidPoint = 681
            while i == 0:
                lastValidPoint -= 1
                i = scandata[lastValidPoint]

            #TODO create filter to ignore outlieers
            extractedLines = self.extractLinesFrom2dDatapoints(scandata, firstValidPoint, lastValidPoint)
            #TODO merge lines
            #The routine uses a standard statistical method, called Chi2- test, to compute a Mahalanobis distance between each pair
            # of line segments based on already computed covariance matrices of line parameters. If 2 line segments have statistical
            # distance less than a threshold, they are merged. T
            walls = self.extractWallsFromLines(extractedLines)

            radians, distance = self.refineWallParameters(walls, scandata)
            walls[0].refinedRadian = radians
            walls[0].refinedDistance = distance
            print("radians: {}, distance: {}".format(walls[0].perpendicularRadian,walls[0].perpendicularDistance))
            print("LR, radians: {}, distance: {}".format(radians,distance))
            
            if self.previousWalls is not None:
                yaw = self.previousWalls[0].perpendicularRadian - walls[0].perpendicularRadian 
                print("yaw angle: {}, last index: {}".format(yaw*180/pi, walls[0].index2))
                self.listOfYaw.append(yaw)
                yawLR = self.calculateYaw(self.previousWalls, walls)
                print("yaw angle LR: {}".format(yawLR*180/pi))
                self.listOfYawLR.append(yawLR)
                #TODO match corner points to previous iteration (by lookign at similar distance and angle)

            self.previousWalls = walls
            self.extractedLinesCountDebug = 0
            self.update()
            sleep(3) #test purpose

        # lengthList = len(self.listOfYaw)
        # if lengthList % 10 == 0 and lengthList> 0:
        #     print("linear regression, average yaw error: {}".format(sum(self.listOfYawLR)/lengthList))
        #     print("no linear regression, average yaw error: {}".format(sum(self.listOfYaw)/lengthList))
        #     sleep(10)
        # Reschedule this task immediately
        self.after(1, self._task)
        
        # Record another display for reporting performance
        self.showcount += 1

    # lets calculate the corner points - split and merge
    def extractLinesFrom2dDatapoints(self, scandata, first, last):
        # print("1index: {}, distance: {}, angle: {}".format(first, scandata[first], self.scanAngles[first]))
        # print("2index: {}, distance: {}, angle: {}".format(last, scandata[last], self.scanAngles[last]))

        x1Raw = self.sinRaw[first] * scandata[first]
        y1Raw = self.cosRaw[first] * scandata[first]
        x2Raw = self.sinRaw[last] * scandata[last]
        y2Raw = self.cosRaw[last] * scandata[last]
        # print("1 x: {}, y: {}".format(x1Raw, y1Raw))
        # print("2 x: {}, y: {}".format(x2Raw, y2Raw))

        firstPointX = self.half_canvas_pix + x1Raw * self.renderScale
        firstPointY = self.half_canvas_pix + -y1Raw * self.renderScale        
        lastPointX = self.half_canvas_pix + x2Raw * self.renderScale
        lastPointY = self.half_canvas_pix + -y2Raw * self.renderScale

        # #test purposes - draw blue line from first point to last point
        # self.extractedLinesCountDebug+=1
        # [self.canvas.coords(self.linesExtracted[self.extractedLinesCountDebug], 
        #                 firstPointX, \
        #                 firstPointY, \
        #                 lastPointX,\
        #                 lastPointY)]    
        # [self.canvas.itemconfig(self.linesExtracted[self.extractedLinesCountDebug], fill=DISPLAY_SCAN_LINE_COLOR_EXTRACTED_LINES)]

        #calculate distance and angle to line drawn through first and last point
        perpendicularRadian, perpendicularDistanceRaw = calculatePerpendicularLine(x1Raw, y1Raw, x2Raw, y2Raw)

        # #test purpose - draw perpendicular line with green
        # perpendicularDistance = perpendicularDistanceRaw * self.renderScale
        # perpendicularLineEndX = self.half_canvas_pix+ perpendicularDistance * sin(perpendicularRadian)
        # perpendicularLineEndY = self.half_canvas_pix+ perpendicularDistance * -cos(perpendicularRadian)
        # perpendicularLineBeginX = self.half_canvas_pix
        # perpendicularLineBeginY = self.half_canvas_pix
        # 
        # self.perpendicularLineCount += 1
        # [self.canvas.coords(self.perpendicularLines[self.perpendicularLineCount], 
        #                 perpendicularLineBeginX, \
        #                 perpendicularLineBeginY, \
        #                 perpendicularLineEndX,\
        #                 perpendicularLineEndY)]    
        # [self.canvas.itemconfig(self.perpendicularLines[self.perpendicularLineCount], fill=DISPLAY_SCAN_LINE_COLOR_PERPENDICULAR)]
    

        largestDistance = 0
        indexLargestDistance = 0
        missingDataCount = 0
        for i in range(first +1, last):
            if scandata[i] != 0:
                # math step 4 on paper, calculate distance from each point to perpendicular line
                distance = fabs(scandata[i] * cos(perpendicularRadian - self.scanRadians[i]) - perpendicularDistanceRaw)
                if distance > largestDistance:
                    indexLargestDistance = i
                    largestDistance = distance
            else:
                missingDataCount+=1

        #threshhold for detecting new corner point (in mm)
        if largestDistance > 30:
            # # test purpose - draw largest distance line in red
            # [self.canvas.itemconfig(self.linesLidar[indexLargestDistance], fill=DISPLAY_SCAN_LINE_COLOR_LARGEST_DISTANCE)]
            # self.update()
            # sleep(0)

            #recursively check for new corner points
            listOfWalls = self.extractLinesFrom2dDatapoints(scandata, first, indexLargestDistance)
            listOfWalls.extend(self.extractLinesFrom2dDatapoints(scandata, indexLargestDistance, last))
        else:
            #(x1,y1,x2,y2,index1,index2,amountOfDataPoints,perpendicularDistance, perpendicularRadian)
            listOfWalls = [extractedLine(firstPointX, firstPointY, lastPointX, lastPointY, x1Raw, y1Raw, x2Raw, y2Raw, first, 
                last, last-first-missingDataCount, perpendicularDistanceRaw, perpendicularRadian)]
        return listOfWalls

    def extractWallsFromLines(self, extractedLines):
        print("wall count: {}, ".format(len(extractedLines)))
            
        def amountOfDataPoints(elem):
            return elem.amountOfDataPoints

        extractedLines.sort(reverse=True, key=amountOfDataPoints)
        #TODO for the biggest wall, calculate the mean line from all the data points (linear regression)
        #TODO best filter for wall selection: one that has the most data points, and variance is small!
        for idx, w in enumerate(extractedLines):
            print("idx: {}, point 1 idx: {}, point 2 idx: {}".format(idx, w.index1, w.index2))
            self.canvas.coords(self.linesExtracted[idx], w.x1, w.y1, w.x2, w.y2)
            widthWall = 1 + 10/(idx +1)
            self.canvas.itemconfig(self.linesExtracted[idx], fill=DISPLAY_SCAN_LINE_COLOR_EXTRACTED_LINES, width=widthWall)
            
        
        return extractedLines

    def calculateYaw(self, previousWalls, walls):
        return previousWalls[0].refinedRadian - walls[0].refinedRadian 


    def refineWallParameters(self, walls, scandata):
        # sliceObject = slice(walls[0].index1,walls[0].index2)
        listX = []
        listY = []
        for i in range(walls[0].index1, walls[0].index2+1):
            print(i)
            listX.append(scandata[i] * self.sinRaw[i])
            listY.append(scandata[i] * self.cosRaw[i])
        print("x1: {}, y1: {}".format(walls[0].x1Raw, walls[0].y1Raw))
        print("x2: {}, y2 :{}".format(walls[0].x2Raw, walls[0].y2Raw))
        print("x1: {}, y1: {}".format(listX[0], listY[0]))
        print("x2: {}, y2 :{}".format(listX[-1], listY[-1]))
        return linearRegression(listX, listY)
        
# Instantiate and pop up the window
if __name__ == '__main__':
    
    plotter = URGPlotter()
    
    plotter.run()
