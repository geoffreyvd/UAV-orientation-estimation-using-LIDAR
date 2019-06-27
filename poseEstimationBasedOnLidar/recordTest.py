#!/usr/bin/env python3
URG_DEVICE                  = '/dev/URG04LX'
LOG_END_OF_DISTANCE = 10001
LOG_END_OF_SCAN = 10002
from breezylidar import URG04LX
import sys

# arguments
# 1 = filename to write to
# 2 = LIDAR iterations (0,1s)

class Writer():
    def __init__(self, fileNameToWrite, iterations):
        self.scandata = []
        self.lidar = URG04LX(URG_DEVICE)
        self.fileNameToWrite = fileNameToWrite
        self.iterations = int(iterations)
        self.iterationCount = 0

        self.readScanFromLidarAndWriteToFile()

    def readScanFromLidarAndWriteToFile(self):
        #write to log file
        self.file = open(self.fileNameToWrite, 'wb') 
        while self.iterationCount < self.iterations:
            scandata = self.lidar.getScan()
            if scandata:
                self.iterationCount += 1
                self.scandata = scandata
                for distance in scandata:
                    distanceByte = distance.to_bytes(2, byteorder='big')
                    self.file.write(distanceByte)
                    self.file.write(LOG_END_OF_DISTANCE.to_bytes(2, byteorder='big'))
                self.file.write(LOG_END_OF_SCAN.to_bytes(2, byteorder='big'))

if __name__ == '__main__':
    fileNameToWrite = sys.argv[1]
    iterations = sys.argv[2]
    writer = Writer(fileNameToWrite, iterations)
