#!/usr/bin/env python3
from inspect import getargspec
from math import sin, cos, radians, atan, atan2, pi, fabs, sqrt

def calculatePerpendicularLine(x1Raw, y1Raw, x2Raw, y2Raw):
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

#total least fitting
def linearRegression(listX, listY):
    n = len(listX)
    #1 calculate mean of x and y
    xTotal = 0
    yTotal = 0
    for i in range(n):
        xTotal+=listX[i]
        yTotal+=listY[i]
    xMean = xTotal/n
    yMean = yTotal/n
    #2. calculate angle
    sumOfCovariance = 0
    sumOfDiffToModel = 0
    for i in range(n):
        errorX = listX[i]-xMean
        errorY = listY[i]-yMean
        sumOfCovariance += errorX*errorY
        sumOfDiffToModel += (errorY**2 - errorX**2)
    perpendicularRadian = 0.5*atan((-2*sumOfCovariance)/sumOfDiffToModel)
     
    #3. r = xMean x cos(angle) + yMean x sin(angle)
    perpendicularDistance = xMean * cos(perpendicularRadian) + yMean * sin(perpendicularRadian)
    return perpendicularRadian, perpendicularDistance

def matchWallsWithNewIteration():
    return 0
    #TODO
    #1. implement wall recognition, so we can estimate yaw angle on average of wall yaw angle displacements
    #2. try retrieving IMU yaw angle from pixhawk (NEXT UP)
    #3. wall recognition basesd on imu data, predict where wall from past iteration will be in new iteration
    #4. check if prediction of the walls are present in the new iteration, if so these are the same walls
    #5. so basically create a loop to check a predicted wall and compare it to all walls and see which it most likely represents

#super class which can be inherited to use constructor parameter names as class variable names
class AutoInit(type):
  def __new__(meta, classname, supers, classdict):
    classdict['__init__'] = autoInitDecorator(classdict['__init__'])
    return type.__new__(meta, classname, supers, classdict)

def autoInitDecorator (toDecoreFun):
  def wrapper(*args):
    
    # ['self', 'first_name', 'last_name', 'birth_date', 'sex', 'address']
    argsnames = getargspec(toDecoreFun)[0]
    
    # the values provided when a new instance is created minus the 'self' reference
    # ['Jonh', 'Doe', '21/06/1990', 'male', '216 Caledonia Street']
    argsvalues = [x for x in args[1:]]
    
    # 'self' -> the reference to the instance
    objref = args[0]
    
    # setting the attribute with the corrisponding values to the instance
    # note I am skipping the 'self' reference
    for x in argsnames[1:]:
     	objref.__setattr__(x,argsvalues.pop(0))
    
  return wrapper

class extractedLine(metaclass=AutoInit):
    '''
    line notation class
    '''
    refinedRadian = 0
    refinedDistance = 0

    def __init__(self, x1, y1, x2, y2, x1Raw, y1Raw, x2Raw, y2Raw, index1, index2, amountOfDataPoints, perpendicularDistance, perpendicularRadian):
        pass

class splitAndMerge():
    def __init__(self, config, lidarvisualiser):
        self.config = config
        self.lidarVisualiser = lidarvisualiser
    
    # lets calculate the corner points - split and merge
    def extractLinesFrom2dDatapoints(self, scandata, first, last):
        x1Raw = self.config.sinRaw[first] * scandata[first]
        y1Raw = self.config.cosRaw[first] * scandata[first]
        x2Raw = self.config.sinRaw[last] * scandata[last]
        y2Raw = self.config.cosRaw[last] * scandata[last]

        firstPointX, firstPointY = self.lidarVisualiser.applyScaleToPoint(x1Raw, y1Raw)   
        lastPointX, lastPointY = self.lidarVisualiser.applyScaleToPoint(x2Raw, y2Raw)  

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
                distance = fabs(scandata[i] * cos(perpendicularRadian - self.config.scanRadians[i]) - perpendicularDistanceRaw)
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

        #TODO best filter for wall selection: one that has the most data points, and variance is small!
        extractedLines.sort(reverse=True, key=lambda x : x.amountOfDataPoints)
        self.lidarVisualiser.plotWalls(extractedLines)        
        return extractedLines
    
    def refineWallParameters(self, walls, scandata):
        listX = []
        listY = []
        for i in range(walls[0].index1, walls[0].index2+1):
            listX.append(scandata[i] * self.config.sinRaw[i])
            listY.append(scandata[i] * self.config.cosRaw[i])
        return linearRegression(listX, listY)