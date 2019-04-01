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
DISPLAY_SCAN_LINE_COLOR_LINES     = 'blue'
DISPLAY_SCAN_LINE_COLOR_WALLS     = 'green'

# URG-04LX specs
URG_MAX_SCAN_DIST_MM        = 4000
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
        self.lines = []
        
        # Create a URG04LX object and connect to it
        self.lidar = URG04LX(URG_DEVICE)
        
        # No scan data to start
        self.scandata = []

        # Pre-compute some values useful for plotting
        #-2*pi/3 +
        
        scan_angle_rad = [radians(-URG_DETECTION_DEG/2 + (float(k)/URG_SCAN_SIZE) * \
                                   URG_DETECTION_DEG) for k in range(URG_SCAN_SIZE)]
        self.scanAngles = [scan_angle_rad[k]*180/pi for k in range(URG_SCAN_SIZE)]
        self.indexLargestDistance = 0
        self.extractLinesCount = 0
        self.wallCount = 0

        self.half_canvas_pix = DISPLAY_CANVAS_SIZE_PIXELS / 2
        self.renderScale = self.half_canvas_pix / float(URG_MAX_SCAN_DIST_MM)


        print("origin x and y: {}, renderScale: {}".format(self.half_canvas_pix, self.renderScale))

        self.cos = [-cos(angle) * self.renderScale for angle in scan_angle_rad]
        self.sin = [ sin(angle) * self.renderScale for angle in scan_angle_rad]
        
        # Add scan lines to canvas, to be modified later        
        self.lines = [self.canvas.create_line(\
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
        self.wallsExtracted = [self.canvas.create_line(\
                         0, \
                         0, \
                         self.sin[k] * 10,\
                         self.cos[k] * 10)
                         for k in range(URG_SCAN_SIZE)]
                         
        [self.canvas.itemconfig(line, fill=DISPLAY_SCAN_LINE_COLOR) for line in self.lines]
        
        # create red lines here for indicating corner points.

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
        # Modify the displayed lines according to the current scan
        [self.canvas.coords(self.lines[k], 
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

            self.extractLines(self.scandata, firstValidPoint, lastValidPoint)
            sleep(20) #test purpose

        # Reschedule this task immediately
        self.after(1, self._task)
        
        # Record another display for reporting performance
        self.showcount += 1

    def extractLines(self, scandata, first, last):
        # lets calculate the corner points
        #split and merge
        # mogelijk TODO cos en sin hier omdraaien (doen ze boven ook), wellicht omdat de assen coutner clockwise zijn, ipv clockwise 
        firstPointXRaw = scandata[first] * self.sin[first]
        firstPointYRaw = scandata[first] * self.cos[first]
        firstPointX = self.half_canvas_pix + firstPointXRaw
        firstPointY = self.half_canvas_pix + firstPointYRaw
        
        lastPointXRaw = scandata[last] * self.sin[last]
        lastPointYRaw = scandata[last] * self.cos[last]
        lastPointX = self.half_canvas_pix + lastPointXRaw
        lastPointY = self.half_canvas_pix + lastPointYRaw

        diffX = lastPointXRaw - firstPointXRaw
        diffY = lastPointYRaw - firstPointYRaw
        self.extractLinesCount+=1
        [self.canvas.coords(self.linesExtracted[self.extractLinesCount], 
                        firstPointX, \
                        firstPointY, \
                        lastPointX,\
                        lastPointY)]    
        [self.canvas.itemconfig(self.linesExtracted[self.extractLinesCount], fill=DISPLAY_SCAN_LINE_COLOR_LINES)]

        slope = 0
        if diffX != 0:
            #step 1 see papier for uitwerking - calculate slope
            slope = diffY / diffX
            #step 2 - calculate perpendicular angle from origon to line
            #possible optimization, dont convert to angles, keep using radius
            # perpendicularRadius = atan2(diffY, diffX)
            # perpendicularAngle = perpendicularRadius * 180/pi
            x1 = sin(self.scanAngles[first]*pi/180) * scandata[first]
            y1 = cos(self.scanAngles[first]*pi/180) * scandata[first]
            x2 = sin(self.scanAngles[last]*pi/180) * scandata[last]
            y2 = cos(self.scanAngles[last]*pi/180) * scandata[last]
            print("1 x: {}, y: {}".format(x1, y1))
            print("2 x: {}, y: {}".format(x2, y2))
            # perpendicularRadius = -atan2(y2-y1, x2-x1)
            slope = (y2 - y1) / (x2 - x1)
            perpendicularRadius = -atan(slope)
            perpendicularAngle = perpendicularRadius * 180/pi
            # if perpendicularAngle < 0:
            #     perpendicularAngle = -180-perpendicularAngle
            #     perpendicularRadius = perpendicularAngle*pi/180
            #step 3 calculate perpendicular distance from origon to line
            perpendicularDistance = (y1-slope*x1)/sqrt(slope*slope+1) * self.renderScale
            
        elif diffY > 0:
            #als de slope infinite is (de twee punt coordinaten staan op dezelfde x waarde)
            perpendicularDistance = lastPointXRaw
            perpendicularRadius = 1,57
            perpendicularAngle = 90 #0
        else:
            perpendicularDistance = lastPointXRaw
            perpendicularRadius = 1,57
            perpendicularAngle = 270

        #TODO check perpendicular distance calc
        # print("1 x: {}, y: {}".format(firstPointXRaw, firstPointYRaw))
        # print("2 x: {}, y: {}".format(lastPointXRaw, lastPointYRaw))
        print("1index: {}, distance: {}, angle: {}".format(first, scandata[first], self.scanAngles[first]))
        print("2index: {}, distance: {}, angle: {}".format(last, scandata[last], self.scanAngles[last]))
        print("perpendicular Angle: {}, Distance: {}".format(perpendicularAngle, perpendicularDistance))

        #test purpose draw perpendicular line
        lineEndX = self.half_canvas_pix+ perpendicularDistance * sin(perpendicularRadius)
        lineEndY = self.half_canvas_pix+ perpendicularDistance * -cos(perpendicularRadius)

        lineBeginX = self.half_canvas_pix
        lineBeginY = self.half_canvas_pix

        # #test purpose draw line that is given by perpendicular angle and distance (walls)
        # lineBeginX = perpendicularDistance * sin(perpendicularAngle*pi/180) * self.renderScale
        # lineBeginY = perpendicularDistance * -cos(perpendicularAngle*pi/180) * self.renderScale

        # lineEndX = lineBeginX + 1200
        # lineEndY = lineBeginY + 1200 *slope

        print("lineBegin x: {}, y: {}".format(lineBeginX, lineBeginY))
        print("lineEnd x: {}, y: {}".format(lineEndX, lineEndY))
        print("slope: {}".format(slope))

        self.wallCount += 1
        [self.canvas.coords(self.wallsExtracted[self.wallCount], 
                        lineBeginX, \
                        lineBeginY, \
                        lineEndX,\
                        lineEndY)]    
        [self.canvas.itemconfig(self.wallsExtracted[self.wallCount], fill=DISPLAY_SCAN_LINE_COLOR_WALLS)]
    
        largestDistance = 0
        indexLargestDistance = 0
        for i in range(first +1, last):
            if scandata[i] != 0:
                # math step 4 on paper, calculate distance from each point to perpendicular line
                distance = fabs(scandata[i] * cos((perpendicularAngle - self.scanAngles[i])*pi/180) - perpendicularDistance)
                if distance > largestDistance:
                    indexLargestDistance = i
                    largestDistance = distance
        #threshhold for (in mm)
        if largestDistance > 100:
            print("largestDistance: {}, indexLargestDistance: {}".format(largestDistance, indexLargestDistance))
            print("-----------------------")
            #draw extracted lines
            [self.canvas.itemconfig(self.lines[indexLargestDistance], fill=DISPLAY_SCAN_LINE_COLOR_LARGEST_DISTANCE)]
            self.update()
            sleep(10)
            self.extractLines(scandata, first, indexLargestDistance)
            self.extractLines(scandata, indexLargestDistance, last)
                
        #lets visualize the corner lines
        # [self.canvas.itemconfig(self.lines[self.indexLargestDistance], fill=DISPLAY_SCAN_LINE_COLOR)]
        self.indexLargestDistance = indexLargestDistance

        #TODO 
        
        
# Instantiate and pop up the window
if __name__ == '__main__':
    
    plotter = URGPlotter()
    
    plotter.run()
