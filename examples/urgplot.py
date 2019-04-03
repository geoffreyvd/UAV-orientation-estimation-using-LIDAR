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

if version[0] == '3':
    import tkinter as tk
    import _thread as thread
else:
    import Tkinter as tk
    import thread

# Runs on its own thread
def grab_scan( obj):
    while True:
        scandata = obj.lidar.getScan()
        if scandata:
            obj.scandata = scandata
            obj.count += 1
            sleep(.01) # pause a tiny amount to allow following check to work
            if not obj.running:
                break
                
                
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

        # Pre-compute some values useful for plotting        
        scan_angle_rad = [radians(-URG_DETECTION_DEG/2 + (float(k)/URG_SCAN_SIZE) * \
                                   URG_DETECTION_DEG) for k in range(URG_SCAN_SIZE)]
        self.scanRadians = scan_angle_rad
        self.scanAngles = [scan_angle_rad[k]*180/pi for k in range(URG_SCAN_SIZE)]
        self.extractedLinesCountDebug = 0
        self.perpendicularLineCount = 0

        self.half_canvas_pix = DISPLAY_CANVAS_SIZE_PIXELS / 2
        self.renderScale = self.half_canvas_pix / float(URG_MAX_SCAN_DIST_MM)
        print("origin x and y: {}, renderScale: {}".format(self.half_canvas_pix, self.renderScale))

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
        [self.canvas.coords(self.linesLidar[k], 
                            self.half_canvas_pix, \
                            self.half_canvas_pix, \
                            self.half_canvas_pix + self.sin[k] * self.scandata[k],\
                            self.half_canvas_pix + self.cos[k] * self.scandata[k]) \
         for k in range(len(self.scandata))]    
        if self.scandata:
            i = 0
            firstValidPoint = -1
            while i == 0:
                firstValidPoint += 1
                i = self.scandata[firstValidPoint]
            
            i = 0
            lastValidPoint = 681
            while i == 0:
                lastValidPoint -= 1
                i = self.scandata[lastValidPoint]

            #TODO create filter to ignore outlieers
            extractedLines = self.extractLinesFrom2dDatapoints(self.scandata, firstValidPoint, lastValidPoint)
            walls = self.extractWallsFromLines(extractedLines)
            
            self.extractedLinesCountDebug = 0
            self.update()
            sleep(30) #test purpose

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
        perpendicularRadian, perpendicularDistanceRaw = self.calculatePerpendicularLine(x1Raw, y1Raw, x2Raw, y2Raw)

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
            #(x1,y1,x2,y2,index1,index2,amountOfDataPoints)
            listOfWalls = [(firstPointX, firstPointY, lastPointX, lastPointY, first, last, last-first-missingDataCount)]
        return listOfWalls

    def extractWallsFromLines(self, extractedLines):
        print("wall count: {}, ".format(len(extractedLines)))
            
        def amountOfDataPoints(elem):
            return elem[6]

        extractedLines.sort(reverse=True, key=amountOfDataPoints)
        #TODO make 2 iterations and calculate angle
        #TODO for the biggest wall, calculate the mean line from all the data points 
        #TODO best filter for wall selection: one that has the most data points, and variance is small!
        for idx, w in enumerate(extractedLines):
            print("idx: {}, point 1 idx: {}, point 2 idx: {}".format(idx, w[4], w[5]))
            self.canvas.coords(self.linesExtracted[idx], w[0], w[1], w[2], w[3])
            widthWall = 1 + 10/(idx +1)
            self.canvas.itemconfig(self.linesExtracted[idx], fill=DISPLAY_SCAN_LINE_COLOR_EXTRACTED_LINES, width=widthWall)
    
    def calculatePerpendicularLine(self, x1Raw, y1Raw, x2Raw, y2Raw):
        diffX = x2Raw - x1Raw
        diffY = y2Raw - y1Raw
        slope = 0
        if diffX != 0:
            #step 1 see papier for uitwerking - calculate slope
            slope = diffY / diffX
            #step 2 - calculate perpendicular angle from origon to line
            perpendicularRadian = -atan(slope)
            #step 3 calculate perpendicular distance from origon to line
            perpendicularDistance = (y1Raw-slope*x1Raw)/sqrt(slope*slope+1)            
        elif diffY > 0:
            #als de slope infinite is (de twee punt coordinaten staan op dezelfde x waarde)
            perpendicularDistance = x2Raw
            perpendicularRadian = 0
        else:
            perpendicularDistance = x2Raw
            perpendicularRadian = pi
        # print("perpendicular Angle: {}, Distance: {}".format(perpendicularAngle, perpendicularDistance))
        # print("slope: {}".format(slope))
        return perpendicularRadian, perpendicularDistance
        
# Instantiate and pop up the window
if __name__ == '__main__':
    
    plotter = URGPlotter()
    
    plotter.run()
