#!/usr/bin/env python3
#TODO fix debugger visual studio code
#NOTE, if this programs returns breezylidar connect error, then just retry 2 times

from math import sin, cos, radians, atan, atan2, pi, fabs, sqrt
from time import time, sleep, ctime
from splitAndMerge import splitAndMerge
from mockLIDAR import URGMocker, READ_FROM_SERIAL, READ_FROM_FILE
from lidarVisualiser import lidarVisualiser
from lidarAndCanvasConfig import lidarAndCanvasConfig
# TODO, use numpy instead of python list

class URGPlotter():
    '''
    UGRPlotter extends tk.Frame to plot Lidar scans.
    '''
    
    def __init__(self):
        '''
        Takes no args.  Maybe we could specify colors, lidar params, etc.
        '''
        self.previousWalls = None

        #test
        self.listOfYaw = []
        self.listOfYawLR =[]
        
        self.config = lidarAndCanvasConfig()
        self.mocker = URGMocker(READ_FROM_FILE)
        self.lidarVisualiser = lidarVisualiser(self.config)
        self.splitAndMerge = splitAndMerge(self.config, self.lidarVisualiser)
        
    def run(self):
        '''
        Call this when you're ready to run.
        '''        
        # Record start time and initiate a count of scans for testing
        self.start_sec = time()        
        
        while True:
            plotter._task()        

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

            extractedLines = self.splitAndMerge.extractLinesFrom2dDatapoints(scandata, firstValidPoint, lastValidPoint)
            #TODO merge lines
            #The routine uses a standard statistical method, called Chi2- test, to compute a Mahalanobis distance between each pair
            # of line segments based on already computed covariance matrices of line parameters. If 2 line segments have statistical
            # distance less than a threshold, they are merged. T
            walls = self.splitAndMerge.extractWallsFromLines(extractedLines)

            radians, distance = self.splitAndMerge.refineWallParameters(walls, scandata)
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
            self.lidarVisualiser.updateGUI()
            sleep(3) #test purpose

        # lengthList = len(self.listOfYaw)
        # if lengthList % 10 == 0 and lengthList> 0:
        #     print("linear regression, average yaw error: {}".format(sum(self.listOfYawLR)/lengthList))
        #     print("no linear regression, average yaw error: {}".format(sum(self.listOfYaw)/lengthList))
        #     sleep(10)

    def calculateYaw(self, previousWalls, walls):
        return previousWalls[0].refinedRadian - walls[0].refinedRadian 
        
# Instantiate and pop up the window
if __name__ == '__main__':
    
    plotter = URGPlotter()
    
    plotter.run()
