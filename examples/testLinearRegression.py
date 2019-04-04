#!/usr/bin/env python3
from splitAndMerge import linearRegression

listX = [3,4,5,6,7,8,9]
listY = [7,7,11,11,15,16,19]
perpendicularRadian, perpendicularDistance =  linearRegression(listX, listY)
print("radian {}, distance: {}".format(perpendicularRadian, perpendicularDistance))
