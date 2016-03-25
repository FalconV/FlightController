#!/usr/bin/python

from __future__ import print_function
from threading import Thread
import Queue
import time
import serial


ser = serial.Serial('/dev/tty.SLAB_USBtoUART', timeout=1, baudrate=115200)
# ser = serial.Serial('/dev/cu.usbmodemFD121', timeout=1, baudrate=9600)


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def send(q):
    s = "0,0,0,0*324*\n"
    state = "T"
    r = 1550
    p = 1550
    t = 1100
    y = 1550

    while True:
        try:
            s = q.get(False)
            if s == "END":
                return
        except Queue.Empty:
            pass

        print ('input: ' + s)
        print('state = ' + state)
        # write to serial
        # ser.write(s)
        # received = ser.readline()
        # print ('input: ' + s)
        if is_number(s) == True:

            s = float(s)
            # ensure input is within range
            if s<1100:
                s = 1100
            elif s>1900:
                s = 1900
            else:
                s = s

            # send appropro data
            if state == "R":
                r = s
            elif state == "P":
                p = s
            elif state == "T":
                t = s
            elif state == "Y":
                y = s
             #at this point we should have sent something to the quad
            s = "%s" % str(int(r)) + ",%s" % str(int(p)) + ",%s" % str(int(t)) + ",%s" % str(int(y))
            #calculate checksum
            checksum = 0
            for c in s:
                checksum += ord(c)
            print ('sending: ' + s + "*%d*" % checksum)
            ser.write(s + "*%d*" % checksum)
            received = ser.readline()
            print ('received ' + received)
            while received != "OK\r\n":
                # if the falcon waited too long for you
                if received == "TIMEOUT\r\n" or received == "MISS\r\n":
                    # repeat yourself
                    print ('resending: ' + s)
                    ser.write(s + "*%d*" % checksum)
                    # print 'RTX'
#                    print ('resending: ' + s)
                else:
                   pass
                # listen to the falcon while you wait
                received = ser.readline()
                print ('received: ' + received)
        elif s == "R":
            state = "R"
            print("state: roll")
        elif s == "P":
            state = "P"
            print("state: pitch")
        elif s == "T":
            state = "T"
            print("state: throttle")
        elif s == "Y":
            state = "Y"
            print("state: yaw")

        time.sleep(1)       # check on this


def get(q):
    s = '0,0,0,0*324*\n'
    while True:
        q.put(s)
        s = raw_input()
        if s == "END\n":
            q.put("END\n")
            return
        # process string
        # expected format: roll,pitch,throttle,yaw
        # checksum = 0
        # for c in s:
        #     checksum += ord(c)
        # s = s + '*' + "%d" % checksum + '*\n'
        # mod the checksum, because ain't nobody got time for large checksums
        # checksum %= 2000
        # q.put(s)


queue = Queue.Queue()
thread1 = Thread(target=send, args=(queue,))
thread2 = Thread(target=get, args=(queue,))

thread1.start()
thread2.start()
thread1.join()
thread2.join()
