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

        self.cosRaw = [ cos(angle) for angle in scan_angle_rad]
        self.sinRaw = [ sin(angle) for angle in scan_angle_rad]
        
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

            #TODO create filter to ignore outlieers
            walls = self.extractLines(self.scandata, firstValidPoint, lastValidPoint)
            print("wall count: {}, ".format(len(walls)))
            
            def amountOfDataPoints(elem):
                return elem[6]
            walls.sort(reverse=True, key=amountOfDataPoints)

            for idx, w in enumerate(walls):
                self.canvas.coords(self.linesExtracted[idx], w[0], w[1], w[2], w[3])
                print("idx: {}, point 1 idx: {}, point 2 idx: {}".format(idx, w[4], w[5]))
                widthWall = 1 + 10/(idx +1)
                self.canvas.itemconfig(self.linesExtracted[idx], fill=DISPLAY_SCAN_LINE_COLOR_LINES, width=widthWall)
                

            self.extractLinesCount = 0
            self.update()
            sleep(30) #test purpose

        # Reschedule this task immediately
        self.after(1, self._task)
        
        # Record another display for reporting performance
        self.showcount += 1

    # lets calculate the corner points - split and merge
    def extractLines(self, scandata, first, last):
        print("1index: {}, distance: {}, angle: {}".format(first, scandata[first], self.scanAngles[first]))
        print("2index: {}, distance: {}, angle: {}".format(last, scandata[last], self.scanAngles[last]))

        x1Raw = self.sinRaw[first] * scandata[first]
        y1Raw = self.cosRaw[first] * scandata[first]
        x2Raw = self.sinRaw[last] * scandata[last]
        y2Raw = self.cosRaw[last] * scandata[last]
        print("1 x: {}, y: {}".format(x1Raw, y1Raw))
        print("2 x: {}, y: {}".format(x2Raw, y2Raw))

        firstPointX = self.half_canvas_pix + x1Raw * self.renderScale
        firstPointY = self.half_canvas_pix + -y1Raw * self.renderScale        
        lastPointX = self.half_canvas_pix + x2Raw * self.renderScale
        lastPointY = self.half_canvas_pix + -y2Raw * self.renderScale

        #draw blue line from first point to last point
        self.extractLinesCount+=1
        [self.canvas.coords(self.linesExtracted[self.extractLinesCount], 
                        firstPointX, \
                        firstPointY, \
                        lastPointX,\
                        lastPointY)]    
        [self.canvas.itemconfig(self.linesExtracted[self.extractLinesCount], fill=DISPLAY_SCAN_LINE_COLOR_LINES)]

        diffX = x2Raw - x1Raw
        diffY = y2Raw - y1Raw
        slope = 0
        if diffX != 0:
            #step 1 see papier for uitwerking - calculate slope
            slope = diffY / diffX
            #step 2 - calculate perpendicular angle from origon to line
            #possible optimization, dont convert to angles, keep using radius
            perpendicularRadius = -atan(slope)
            perpendicularAngle = perpendicularRadius * 180/pi
            #step 3 calculate perpendicular distance from origon to line
            perpendicularDistance = (y1Raw-slope*x1Raw)/sqrt(slope*slope+1)            
        elif diffY > 0:
            #als de slope infinite is (de twee punt coordinaten staan op dezelfde x waarde)
            perpendicularDistance = x2Raw
            perpendicularRadius = 0
            perpendicularAngle = 0
        else:
            perpendicularDistance = x2Raw
            perpendicularRadius = pi
            perpendicularAngle = 180

        perpendicularDistanceRaw = perpendicularDistance
        perpendicularDistance *= self.renderScale
        print("perpendicular Angle: {}, Distance: {}".format(perpendicularAngle, perpendicularDistance))
        print("slope: {}".format(slope))

        #test purpose draw perpendicular line
        perpendicularLineEndX = self.half_canvas_pix+ perpendicularDistance * sin(perpendicularRadius)
        perpendicularLineEndY = self.half_canvas_pix+ perpendicularDistance * -cos(perpendicularRadius)
        perpendicularLineBeginX = self.half_canvas_pix
        perpendicularLineBeginY = self.half_canvas_pix

        #print("PerpendicularLineBegin x: {}, y: {}".format(perpendicularLineBeginX, perpendicularLineBeginY))
        print("PerpendicularLineEnd x: {}, y: {}".format(perpendicularLineEndX, perpendicularLineEndY))

        # draw perpendicular line with green
        self.wallCount += 1
        [self.canvas.coords(self.wallsExtracted[self.wallCount], 
                        perpendicularLineBeginX, \
                        perpendicularLineBeginY, \
                        perpendicularLineEndX,\
                        perpendicularLineEndY)]    
        [self.canvas.itemconfig(self.wallsExtracted[self.wallCount], fill=DISPLAY_SCAN_LINE_COLOR_WALLS)]
    
        largestDistance = 0
        indexLargestDistance = 0
        missingDataCount = 0
        for i in range(first +1, last):
            if scandata[i] != 0:
                # math step 4 on paper, calculate distance from each point to perpendicular line
                distance = fabs(scandata[i] * cos((perpendicularAngle - self.scanAngles[i])*pi/180) - perpendicularDistanceRaw)
                if distance > largestDistance:
                    indexLargestDistance = i
                    largestDistance = distance
            else:
                missingDataCount+=1
        #threshhold for (in mm)
        if largestDistance > 30:
            print("largestDistance: {}, indexLargestDistance: {}".format(largestDistance, indexLargestDistance))
            print("-----------------------")
            #draw largest distance line in red
            [self.canvas.itemconfig(self.lines[indexLargestDistance], fill=DISPLAY_SCAN_LINE_COLOR_LARGEST_DISTANCE)]
            self.update()
            sleep(0)
            listOfWalls = self.extractLines(scandata, first, indexLargestDistance)
            listOfWalls.extend(self.extractLines(scandata, indexLargestDistance, last))
        else:
            #(x1,y1,x2,y2,index1,index2,amountOfDataPoints)
            listOfWalls = [(firstPointX, firstPointY, lastPointX, lastPointY, first, last, last-first-missingDataCount)]
            print("-----------------------")
            print("this was the largest line")
            sleep(0)
                
        #lets visualize the corner lines
        # [self.canvas.itemconfig(self.lines[self.indexLargestDistance], fill=DISPLAY_SCAN_LINE_COLOR)]
        self.indexLargestDistance = indexLargestDistance
        return listOfWalls
        
        
# Instantiate and pop up the window
if __name__ == '__main__':
    
    plotter = URGPlotter()
    
    plotter.run()
