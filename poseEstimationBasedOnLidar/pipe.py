#!/usr/bin/env python3
from multiprocessing import Process, Pipe
from time import time, sleep, ctime

def parallelFuncImuYaw(conn):
    while 1:
        conn.send(i)
        sleep(.02)
    conn.close()

if __name__ == '__main__':
    parent_conn, child_conn = Pipe()
    processPipeImuYaw = Process(target=parallelFuncImuYaw, args=(child_conn,))
    processPipeImuYaw.start()
    value = "leeg"
    for i in range (0, 20):
        while parent_conn.poll():
            value = parent_conn.recv()
        print("latest vlaue: {}".format(value))
        sleep(0.1)
    processPipeImuYaw.join() # wait for child process to end
    