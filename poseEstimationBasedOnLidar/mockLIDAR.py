#!/usr/bin/env python3
URG_DEVICE                  = '/dev/URG04LX'
#static usb refernece for URG04LX:
#idVendor: 15d1
#idProduct: 0000

LOG_END_OF_DISTANCE = 10001
LOG_END_OF_SCAN = 10002
READ_FROM_SERIAL = 1
READ_FROM_FILE = 2

from breezylidar import URG04LX

from time import time, sleep, ctime
from sys import exit, version
                
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
            self.file = open('test', 'rb') 
            self.bytesFromLIDAR = self.file.read()
            self.scanIndex = 0         
        elif readFrom == READ_FROM_SERIAL:
            # Create a URG04LX object and connect to it
            self.lidar = URG04LX(URG_DEVICE)
            #thread.start_new_thread( grab_scan, (self,) ) 
            
    def _quit(self):
        del self.lidar
        exit(0)
     
    def getScan(self):
        self.scanCount += 1
        if self.readFrom == READ_FROM_FILE:
            scandata = []
            i=0
            for i in range(self.scanIndex, len(self.bytesFromLIDAR), 2):
                parsedInt = int.from_bytes([self.bytesFromLIDAR[i], self.bytesFromLIDAR[i+1]] , 'big')
                if parsedInt == LOG_END_OF_DISTANCE:
                    continue
                elif parsedInt == LOG_END_OF_SCAN:
                    break
                else:
                    scandata.append(parsedInt)
            self.scanIndex = i + 2   
            return scandata
        else:
            return self.lidar.getScan()

    def getCount(self):
        return self.scanCount

    def exitLidar(self):
        if self.readFrom == READ_FROM_SERIAL:
            del self.lidar
            
# Instantiate
if __name__ == '__main__':
    
    mocker = URGMocker(3)