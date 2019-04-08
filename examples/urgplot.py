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

#TODO
#1. implement wall recognition, so we can estimate yaw angle on average of wall yaw angle displacements
#2. try retrieving IMU yaw angle from pixhawk
#3. wall recognition basesd on imu data, predict where wall from past iteration will be in new iteration
#4. check if prediction of the walls are present in the new iteration, if so these are the same walls
#5. so basically create a loop to check a predicted wall and compare it to all walls and see which it most likely represents

#NOTE, if this programs returns breezylidar connect error, then just retry 2 times

# URG-04LX specs
URG_MAX_SCAN_DIST_MM        = 2000
URG_DETECTION_DEG           = 240
URG_SCAN_SIZE               = 682

from math import sin, cos, radians, atan, atan2, pi, fabs, sqrt
from time import time, sleep, ctime
from splitAndMerge import extractedLine, calculatePerpendicularLine, linearRegression
from mockLIDAR import URGMocker, READ_FROM_SERIAL, READ_FROM_FILE
from lidarVisualiser import lidarVisualiser, DISPLAY_CANVAS_SIZE_PIXELS
# TODO, use numpy instead of python list
    
def multiplication(a, b): #TODO make lambda
    return a*b

class URGPlotter():
    '''
    UGRPlotter extends tk.Frame to plot Lidar scans.
    '''
    
    def __init__(self):
        '''
        Takes no args.  Maybe we could specify colors, lidar params, etc.
        '''

        # No scanlines initially                             
        self.linesLidar = []

        self.extractedLinesCountDebug = 0
        self.perpendicularLineCount = 0
        self.previousWalls = None

        #test
        self.listOfYaw = []
        self.listOfYawLR =[]
        
        self.mocker = URGMocker(READ_FROM_FILE)
        self.lidarVisualiser = lidarVisualiser()        
        self.renderScale = self.lidarVisualiser.renderScale
        self.half_canvas_pix = DISPLAY_CANVAS_SIZE_PIXELS/2

        # Pre-compute some values useful for plotting        
        scan_angle_rad = [radians(-URG_DETECTION_DEG/2 + (float(k)/URG_SCAN_SIZE) * \
                                   URG_DETECTION_DEG) for k in range(URG_SCAN_SIZE)]
        self.scanRadians = scan_angle_rad
        self.scanAngles = [scan_angle_rad[k]*180/pi for k in range(URG_SCAN_SIZE)]

        self.cos = [-cos(angle) * self.renderScale for angle in scan_angle_rad]
        self.sin = [ sin(angle) * self.renderScale for angle in scan_angle_rad]
        self.cosRaw = [ cos(angle) for angle in scan_angle_rad]
        self.sinRaw = [ sin(angle) for angle in scan_angle_rad]        
        
    def run(self):
        '''
        Call this when you're ready to run.
        '''        
        # Record start time and initiate a count of scans for testing
        self.start_sec = time()        
        # self.lidarVisualiser.startGUI() 
        # Start the recursive timer-task
        while True:
            plotter._task()        
        
    def destroy(self):
        '''
        Called automagically when user clicks X to close window.
        '''  

        self._quit()

    def _quit(self):
        elapsed_sec = time() - self.start_sec
        scanCount = self.mocker.getCount()
        showCount = self.lidarVisualiser.getShowCount()
        print('%d scans    in %f sec = %f scans/sec' % (scanCount, elapsed_sec, scanCount/elapsed_sec))
        print('%d displays in %f sec = %f displays/sec' % (showCount, elapsed_sec, showCount/elapsed_sec))
        # exit(0)

    def _task(self):
        # The only thing here that has to happen is setting the lengh of the line, by setting a new endpoint for the line 
        # x = cos * scanPointDistance, y = sin * scanPointDistance
        scandata = self.mocker.getScan()
        if scandata:
            self.lidarVisualiser.plotScanDataPointsAsLines(scandata)
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
            #TODO teken de lr lijn, volgens mij is het dikke poep
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
            self.lidarVisualiser.updateGUI()
            sleep(3) #test purpose

        # lengthList = len(self.listOfYaw)
        # if lengthList % 10 == 0 and lengthList> 0:
        #     print("linear regression, average yaw error: {}".format(sum(self.listOfYawLR)/lengthList))
        #     print("no linear regression, average yaw error: {}".format(sum(self.listOfYaw)/lengthList))
        #     sleep(10)
        # Reschedule this task immediately
        # self.after(1, self._task)

    # lets calculate the corner points - split and merge
    def extractLinesFrom2dDatapoints(self, scandata, first, last):
        x1Raw = self.sinRaw[first] * scandata[first]
        y1Raw = self.cosRaw[first] * scandata[first]
        x2Raw = self.sinRaw[last] * scandata[last]
        y2Raw = self.cosRaw[last] * scandata[last]

        firstPointX = self.half_canvas_pix + x1Raw * self.renderScale
        firstPointY = self.half_canvas_pix + -y1Raw * self.renderScale        
        lastPointX = self.half_canvas_pix + x2Raw * self.renderScale
        lastPointY = self.half_canvas_pix + -y2Raw * self.renderScale

        # #test purposes - draw blue line from first point to last point
        # self.lidarVisualiser.plotSplitLine(firstPointX, firstPointY, lastPointX, lastPointY)

        #calculate distance and angle to line drawn through first and last point
        perpendicularRadian, perpendicularDistanceRaw = calculatePerpendicularLine(x1Raw, y1Raw, x2Raw, y2Raw)

        # #test purpose - draw perpendicular line with green 
        # self.lidarVisualiser.plotPerpendicularLines(perpendicularDistance, perpendicularRadian)    

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
            # self.lidarVisualiser.plotLargestDistance(indexLargestDistance)
            
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
        self.lidarVisualiser.plotWalls(extractedLines)        
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
