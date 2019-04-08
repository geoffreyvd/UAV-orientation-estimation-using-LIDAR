#!/usr/bin/env python3
URG_DEVICE                  = '/dev/ttyACM0'

LOG_END_OF_DISTANCE = 10001
LOG_END_OF_SCAN = 10002
READ_FROM_SERIAL = 1
READ_FROM_FILE = 2

from breezylidar import URG04LX

from time import time, sleep, ctime
from sys import exit, version

if version[0] == '3':
    import _thread as thread
else:
    import thread

# Runs on its own thread
def grab_scan(obj):
    while True:
        scandata = obj.lidar.getScan()
        if scandata:
            obj.scandata = scandata
            obj.scanCount += 1
            sleep(.01) # pause a tiny amount to allow following check to work
            if not obj.running:
                break
                
class URGMocker():
    '''
    
    '''
    def __init__(self, readFrom):
        '''
        Takes no args.  Maybe we could specify colors, lidar params, etc.
        '''
        self.bytesFromLIDAR = []
        # No scan data to start
        self.scandata = []
        self.scanCount = 0
        self.running = True

        self.readFrom = readFrom
        if readFrom == READ_FROM_FILE:
            ##read from log file
            self.file = open('urg04-LX-log', 'rb') 
            self.bytesFromLIDAR = self.file.read()
            self.scanIndex = 0         
        elif readFrom == READ_FROM_SERIAL:
            # Create a URG04LX object and connect to it
            self.lidar = URG04LX(URG_DEVICE)
            thread.start_new_thread( grab_scan, (self,) ) 
        else:
            self.lidar = URG04LX(URG_DEVICE)
            self.readScanFromLidarAndWriteToFile()
            
    def _quit(self):
        del self.lidar
        exit(0)
     
    def getScan(self):
        if self.readFrom == READ_FROM_FILE:
            scandata = []
            i=0
            for i in range(self.scanIndex, len(self.bytesFromLIDAR), 2):
                parsedInt = int.from_bytes([self.bytesFromLIDAR[i], self.bytesFromLIDAR[i+1]] , 'big')
                print("parsed int: {}".format(parsedInt))
                print("i: {}".format(i))
                if parsedInt == LOG_END_OF_DISTANCE:
                    continue
                elif parsedInt == LOG_END_OF_SCAN:
                    break
                else:
                    scandata.append(parsedInt)
            print("scanindex: {}".format(self.scanIndex)) 
            self.scanIndex = i + 2   
            return scandata
        else:
            return self.scandata

    def getCount(self):
        return self.scanCount

    def readScanFromLidarAndWriteToFile(self):
        #write to log file
        self.file = open('urg04-LX-log1', 'wb') 
        while True:
            scandata = self.lidar.getScan()
            if scandata:
                self.scandata = scandata
                for distance in scandata:
                    distanceByte = distance.to_bytes(2, byteorder='big')
                    self.file.write(distanceByte)
                    self.file.write(LOG_END_OF_DISTANCE.to_bytes(2, byteorder='big'))
                self.file.write(LOG_END_OF_SCAN.to_bytes(2, byteorder='big'))
                sleep(.01) # pause a tiny amount to allow following check to work
                if not self.running:
                    break
        
# Instantiate
if __name__ == '__main__':
    
    mocker = URGMocker(3)