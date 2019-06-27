#!/usr/bin/env python3

#apt-get install python3-tk
import rospy
import numpy as np
from plot import plotLidar, plotYaw, plotPosition
from math import sin, cos, radians, atan, atan2, pi, fabs, sqrt
from time import time, sleep, ctime
from splitAndMerge import SplitAndMerge, mergeCollinearlines, filterLines, calculatePositionDisplacement, determinePosition
from mockLIDAR import URGMocker, READ_FROM_SERIAL, READ_FROM_FILE
from lidarVisualiser import lidarVisualiser
from lidarAndCanvasConfig import lidarAndCanvasConfig
from pixhawkWrapper import pixhawk
from operator import attrgetter, methodcaller
import signal
import sys
from kalmanFilter import KF
# TODO, use numpy instead of python list

MATCH_WALLS_MAXIMUM_ANGLE = 0.09 #radians

END_AFTER_ITERATIONS = 6000

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
        self.listOfYawSum = [0]
        self.listOfAverageYaw = [0]
        self.listOfAverageYawSum = []
        self.listOfImuYaw = []
        self.lidarErrors = 0

        self.lidarYawStart = 999
        self.lidarYawEnd = 0
        self.changedWallCount = 0
        self.minimumScanDistance = 250 #mm
        self.wallMapping = []

        self.positionX = 0
        self.positionY = 0
        self.listOfX = []
        self.listOfY = []
        self.firstX = 0
        self.firstY = 0

        self.listOfImuTest = []
        self.imuYawBiasCompensated = []

        self.changedWall = True
        self.config = lidarAndCanvasConfig()
        self.mocker = URGMocker(READ_FROM_SERIAL)
        self.pixhawk4 = pixhawk(READ_FROM_SERIAL)
        self.lidarVisualiser = lidarVisualiser(self.config)
        self.splitAndMerge = SplitAndMerge(self.config, self.lidarVisualiser)
        self.kf = KF()

        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
        
    def run(self):
        '''
        Call this when you're ready to run.
        '''        
        # Record start time and initiate a count of scans for testing
        self.start_sec = time()
        #register interupt handler for cleaning up parallel processes
        signal.signal(signal.SIGINT, self.signal_handler)

        while True:
            plotter._task()    
            sleep(0.01)    

    def _quit(self):
        elapsed_sec = time() - self.start_sec
        scanCount = self.mocker.getCount()
        showCount = self.lidarVisualiser.getShowCount()
        print('%d scans    in %f sec = %f scans/sec' % (scanCount, elapsed_sec, scanCount/elapsed_sec))
        print('%d displays in %f sec = %f displays/sec' % (showCount, elapsed_sec, showCount/elapsed_sec))
        self.mocker.exitLidar()
        self.pixhawk4.closeParallelProcess()
        # plotLidar(self.listOfYawSum, self.listOfAverageYawSum, self.listOfImuYawSum)
        listOfImuYawSum = self.pixhawk4.getImuYawSumList()
        plotYaw(self.listOfYawSum, listOfImuYawSum, self.kf.getEstimatedStates(), self.kf.getEstimatedStatesBias(), self.imuYawBiasCompensated)
        # plotYaw(self.listOfYawSum, self.listOfAverageYawSum, self.listOfImuYawSum)
        # plotPosition(self.listOfX, self.listOfY)
        exit(0)

    def signal_handler(self, sig, frame):
        self._quit()

    def state_callback(self, msg):
        print('state schange: {}', format(msg))

    def _task(self):
        # The only thing here that has to happen is setting the lengh of the line, by setting a new endpoint for the line 
        # x = cos * scanPointDistance, y = sin * scanPointDistance
        scandata = self.mocker.getScan()
        if scandata:
            lengthList = len(self.listOfYaw)
            startTimeIteration = time()
            amountOfFalseData = 0
            for i, point in enumerate(scandata):
                if point < self.minimumScanDistance:
                    scandata[i] =0
                    amountOfFalseData+=1
            # print("[{}], {}".format(startTimeIteration - self.start_sec, lengthList))
            self.lidarVisualiser.plotScanDataPointsAsLines(scandata)

            #skip scan if scan has not enough datapoints
            if amountOfFalseData < 630:
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
                extractedLines = filterLines(extractedLines)
                #test sort before merge, so that bigger walls get merged first, might result in better results
                extractedLines = sorted(extractedLines, reverse=True, key=attrgetter('score')) 
                print("walls befor emerge: {}".format(len(extractedLines)))
                mergedLines = mergeCollinearlines(extractedLines)
                self.debugingWallsBeforeFilter = extractedLines
                print("walls after merge: {}".format(len(mergedLines)))
                walls = self.splitAndMerge.extractWallsFromLines(mergedLines)
                self.lidarVisualiser.updateGUI()
            else:
                print("not enough datapoints")

            # linear regression didnt seem to be necessary
            # walls[0].refinedRadian, walls[0].refinedDistance = self.splitAndMerge.refineWallParameters(walls, scandata)

            imuYaw = self.pixhawk4.getImuYawDisplacement() #radian
            listOfImuYawRatesBetweenLidarScan = self.pixhawk4.getImuYawRates()
            imuyaws = self.pixhawk4.getImuYaws()
            for idx, zAngularVelocity in enumerate(listOfImuYawRatesBetweenLidarScan):
                x = self.kf.predict(zAngularVelocity)
                if self.imuYawBiasCompensated != []:
                    self.imuYawBiasCompensated.append(self.imuYawBiasCompensated[-1] + imuyaws[idx]/pi*180 - 0.02* x[1])
                else:
                    self.imuYawBiasCompensated.append(imuyaws[idx]/pi*180 + 0.02 * x[1])
            
            if amountOfFalseData < 630:
                #calculate yaw
                lidarYaw = self.calculateYaw(walls, imuYaw)
                if lidarYaw != None:    
                    if lidarYaw != 0:
                        lidarYaw /=180
                        lidarYaw *= pi
                        print("estimated state: {}".format(self.kf.update(lidarYaw)))

                #caculate position
                if self.wallMapping != []:
                    if self.firstX == 0:
                        self.firstX, self.firstY = calculatePositionDisplacement(walls[0].perpendicularRadian, walls[0].perpendicularDistance, 0)
                    if self.listOfYawSum != [] and walls != []:
                        x, y = determinePosition(walls, self.previousWalls, self.wallMapping, self.listOfYawSum[-1]/180*pi)
                        self.positionX += x
                        self.positionY += y
                        self.listOfX.append(self.positionX)
                        self.listOfY.append(self.positionY)
                        print("x: {}, y: {}".format(self.positionX, self.positionY))
                    plotPosition(self.listOfX, self.listOfY)
                self.previousWalls = walls
            else:
                self.listOfYaw.append(self.listOfYaw[-1])
                self.listOfAverageYaw.append(self.listOfAverageYaw[-1])
                self.listOfImuYaw.append(imuYaw)
                self.listOfYawSum.append(self.listOfYawSum[-1])

            if lengthList % END_AFTER_ITERATIONS == 0 and lengthList> 0:
                print("no linear regression, yaw (error when lidar stood still): {}".format(sum(self.listOfYaw)))
                print("IMU yaw error: {}".format(sum(self.listOfImuYaw)))
                print("yaw from wall 0 from start to end: {}".format(self.lidarYawEnd - self.lidarYawStart))
                print("times lidar couldnt provide yaw: {}".format(self.lidarErrors))
                print("changedwallsCount: {}".format(self.changedWallCount))
                x,y = calculatePositionDisplacement(walls[0].perpendicularRadian, walls[0].perpendicularDistance, 0)

                print("distacen x: {}, distance y: {}".format(self.firstX - x, self.firstY -y))
                self._quit()
    
    #TODO refact to splitandmerge class
    def calculateYaw(self, walls, imuYaw):
        if self.previousWalls is not None and imuYaw is not None:
            wallMapping = [] #tuples with (previouswall, wall)
            estimatedWalls = []
            #wall matching!
            #estimated walls = add yaw to all walls from previous iteraiton
            for i in range(0, len(self.previousWalls)):
                estimatedWalls.append(self.previousWalls[i].perpendicularRadian + imuYaw)
                # TODO distance + imu translation
                smallestYawDiff = 9999
                smallestYawDiffIndex = -1
                for j in range(0, len(walls)):
                    #substract current wall angles and distances from estimated wall angles and distances
                    yawDiff = abs(estimatedWalls[i] - walls[j].perpendicularRadian)
                    if yawDiff < smallestYawDiff:
                        smallestYawDiff = yawDiff
                        smallestYawDiffIndex = j
                #take the smallest difference, and if smaller than a certain threshhold, the 2 walls match
                if smallestYawDiffIndex != -1 and smallestYawDiff < MATCH_WALLS_MAXIMUM_ANGLE:
                    #threshold based on IMU uncertainty/error after 100ms + lidar error
                    wallMapping.append((i, smallestYawDiffIndex))

            #if walls have been matched
            if len(wallMapping) != 0:
                averageLidarYaw = 0
                totalScore = 0
                biggestScore =0
                biggestMatchedWall = (0,0)
                #calculate total score so to devide yaw chnages per wall based on score for the averageYaw
                for map in wallMapping:
                    score = walls[map[1]].score + self.previousWalls[map[0]].score
                    if map[0]==0:
                        score *= 2.5 #give previous biggest wall a 20 percent bigger score so it doesnt keep switching 
                    if score > biggestScore:
                        biggestScore = score
                        biggestMatchedWall = map
                    totalScore += score
                scoreScale = 1.0/ totalScore
                for map in wallMapping:
                    mappedWallYawDiff = walls[map[1]].perpendicularRadian -self.previousWalls[map[0]].perpendicularRadian
                    score = walls[map[1]].score + self.previousWalls[map[0]].score
                    averageLidarYaw += mappedWallYawDiff * scoreScale * score
                averageLidarYaw /= len(wallMapping)
                lidarYaw = walls[biggestMatchedWall[1]].perpendicularRadian - self.previousWalls[biggestMatchedWall[0]].perpendicularRadian
                if biggestMatchedWall[0] != 0:
                    self.changedWallCount += 1
                
                # update wall list if the previous biggest wall with the extra 200 percent got to be the biggest again
                if biggestMatchedWall[1] != 0:
                    temp=walls[0]
                    walls[0]= walls[biggestMatchedWall[1]]
                    walls[biggestMatchedWall[1]] = temp
                    #change wallmaping according to swap
                    for idx, map in enumerate(wallMapping):
                        if map == biggestMatchedWall:
                            wallMapping[idx] = (0,0)
                        elif map[1] == 0:
                            wallMapping[idx] = (map[0], biggestMatchedWall[1])
                    biggestMatchedWall = (0,0)
                self.wallMapping = wallMapping            

                #add yaw data to list to plot
                lidarYawDegree = lidarYaw*180/pi
                if self.lidarYawStart == 999:
                    self.lidarYawStart = lidarYaw
                self.lidarYawEnd = lidarYaw
                averageLidarYawDegree = averageLidarYaw*180/pi
                imuYawDegree = imuYaw*180/pi
                # if abs(lidarYawDegree) > 0.5: #debugging
                #     print(wallMapping)
                #     print(self.previousWalls[wallMapping[0][0]].amountOfDataPoints)
                #     print(walls[wallMapping[0][1]].amountOfDataPoints)
                self.listOfYaw.append(lidarYawDegree)
                self.listOfAverageYaw.append(averageLidarYawDegree)
                self.listOfImuYaw.append(imuYawDegree)

                self.listOfYawSum.append(self.listOfYawSum[-1] + lidarYawDegree)
                # self.listOfAverageYawSum.append(self.listOfAverageYawSum[-1] + averageLidarYawDegree)
                self.lidarVisualiser.displayYaw(self.listOfYawSum[-1], self.pixhawk4.getYawSum()*180/pi)
                print("wall count: {}, wallMatches: {}".format(len(walls), len(wallMapping)))
                return self.listOfYawSum[-1]
            else:
                print("no mapping found!! unable to provide yaw estimate")

                self.listOfYaw.append(self.listOfYaw[-1])
                self.listOfAverageYaw.append(self.listOfAverageYaw[-1])
                self.listOfImuYaw.append(imuYaw)
                self.listOfYawSum.append(self.listOfYawSum[-1])

                self.wallMapping = []
                self.lidarErrors+=1
                return 0
                
    # daarna mogelijk:
	# -aan de hand van VICON ofzo de positie estimate doorgeven aan de FCU (zodat hij zich zel kan corrigeren)
	# -set local position om te vliegen
	# -kalman filter

# Instantiate and pop up the window
if __name__ == '__main__':
	rospy.init_node("obstacle_detection", anonymous=True)
    plotter = URGPlotter()
    
    plotter.run()
