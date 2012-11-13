# -*- coding: Shift-JIS -*-
"""
"""
import os
from os import *
from ctypes import *

class Roomba:
    """
    """
    def __init__(self, portName, baudrate):
        """
        Constructor
        """
        print 'Init'
        if (os.name == 'nt') : # Windows XP
            if(os.path.isfile('libRoomba.dll')):
                print 'OK'
                self.lib = cdll.LoadLibrary('libRoomba.dll')    
            elif(os.path.isdir(os.environ.get('windir', '') + os.sep + "SysWOW64")):
                dllpath = os.environ.get('windir', '') + os.sep + "SysWOW64" + os.sep + "libRoomba.dll"
                if(os.path.isfile(dllpath)):
                    self.lib = cdll.LoadLibrary(dllpath)
                else:
                    dllpath = os.environ.get('windir', '') + os.sep + "System32" + os.sep + "libRoomba.dll"             
                    self.lib = cdll.LoadLibrary(dllpath)
            else:
                dllpath = os.environ.get('windir', '') + os.sep + "System32" + os.sep + "libRoomba.dll"
                self.lib = cdll.LoadLibrary(dllpath)
        else: # Linux
            dllpath = "/usr/lib/libRoomba.so"
            self.lib = cdll.LoadLibrary(dllpath)

        self.handle = self.lib.Roomba_create(portName, baudrate);

	self.MODEL_CREATE = 0
	self.MODEL_500SERIES = 1

        self.MODE_OFF = 90
        self.MODE_PASSIVE = 100
        self.MODE_SAFE = 101
        self.MODE_FULL = 102
        self.MODE_SLEEP = 103
        self.MODE_SPOT_CLEAN = 104
        self.MODE_NORMAL_CLEAN = 105
        self.MODE_MAX_TIME_CLEAN = 106
        self.MODE_DOCK = 107
        self.MODE_POWER_DOWN = 108


        self.LED_CHECK_ROBOT = 8
        self.LED_DOCK = 4
        self.LED_SPOT = 2
        self.LED_DEBRIS = 1


        self.MOTOR_CCW = -1
        self.MOTOR_OFF = 0
        self.MOTOR_CW = 1
        self.MOTOR_ON = 1

        self.SideBrush = 1
        self.Vacuum = 2
        self.MainBrush = 4
        self.SideBrushOpposite = 8
        self.MainBrushOpposite = 16
        self.LeftWheel = 16
        self.RightWheel = 8

        self.Clock = 128
        self.Schedule = 64
        self.Day = 32
        self.Hour = 16
        self.Minute = 8
        self.Dock = 4
        self.Spot = 2
        self.Clean = 1


        self.NotCharging = 0
        self.ReconditioningCharging = 1
        self.FullCharging = 2
        self.TrickleCharging = 3
        self.Waiting = 4
        self.ChargingFaultCondition = 5

    def setMode(self, mode):
        self.lib.Roomba_setMode(self.handle, c_int(mode))

    def getMode(self):
        return self.lib.Roomba_getMode(self.handle)

    def start(self):
        self.lib.Roomba_start(self.handle)

    def clean(self):
        self.lib.Roomba_clean(self.handle)

    def maxClean(self):
        self.lib.Roomba_maxClean(self.handle)

    def spotClean(self):
        self.lib.Roomba_spotClean(self.handle)

    def powerDown(self):
        self.lib.Roomba_powerDown(self.handle)

    def dock(self):
        self.lib.Roomba_dock(self.handle)

    def fullControl(self):
        self.lib.Roomba_fullControl(self.handle)

    def safeControl(self):
        self.lib.Roomba_safeControl(self.handle)

    def drive(self, translationalVelocity, turnRadius):
        self.lib.Roomba_drive(self.handle, c_short(translationalVelocity), c_short(turnRadius))

    def driveDirect(self, rightWheelVelocity, leftWheelVelocity):
        self.lib.Roomba_driveDirect(self.handle, c_short(rightWheelVelocity), c_short(leftWheelVelocity))

    def drivePWM(self, rightWheel, leftWheel):
        self.lib.Roomba_drivePWM(self.handle, c_short(rightWheel), c_short(leftWheel))

    def driveMotors(self, mainBrush, sideBrush, vacuum):
        self.lib.Roomba_driveMotors(self.handle, mainBrush, sideBrush, vacuum)
    
    def setLED(self, leds, intensity, color):
        self.lib.Roomba_setLED(self.handle, c_ubyte(leds), c_ubyte(intensity), c_ubyte(color))

    def runAsync(self):
        self.lib.Roomba_runAsync(self.handle)

    def isRightWheelDropped(self):
        flag = c_int(0)
        self.lib.Roomba_isRightWheelDropped(self.handle, byref(flag))
        return true if flag == 0 else false
    
    def isLeftWheelDropped(self):
        flag = c_int(0)
        self.lib.Roomba_isLeftWheelDropped(self.handle, byref(flag))
        return true if flag == 0 else false
    
    def isRightBump(self):
        flag = c_int(0)
        self.lib.Roomba_isRightBump(self.handle, byref(flag))
        return true if flag == 0 else false

    def isLeftBump(self):
        flag = c_int(0)
        self.lib.Roomba_isLeftBump(self.handle, byref(flag))
        return true if flag == 0 else false

    def isCliffLeft(self):
        flag = c_int(0)
        self.lib.Roomba_isCliffLeft(self.handle, byref(flag))
        return true if flag == 0 else false

    def isCliffFrontLeft(self):
        flag = c_int(0)
        self.lib.Roomba_isCliffFrontLeft(self.handle, byref(flag))
        return true if flag == 0 else false

    def isCliffFrontRight(self):
        flag = c_int(0)
        self.lib.Roomba_isCliffFrontRight(self.handle, byref(flag))
        return true if flag == 0 else false

    def isCliffRight(self):
        flag = c_int(0)
        self.lib.Roomba_isCliffRight(self.handle, byref(flag))
        return true if flag == 0 else false

    def isVirtualWall(self):
        flag = c_int(0)
        self.lib.Roomba_isVirtualWall(self.handle, byref(flag))
        return true if flag == 0 else false

    """
    def isWheelOvercurrents(self):
        return self.lib.Roomba_isWheelOvercurrents(self.handle) == 0 ? false :true
    
    def isRightWheelOvercurrents(self):
        return self.lib.Roomba_isRightWheelOvercurrent(self.handle) == 0 ? false :true
    
    def isLeftWheelOvercurrents(self):
        return self.lib.Roomba_isLeftWheelOvercurrent(self.handle) == 0 ? false :true

    def isMainBrushOvercurrents(self):
        return self.lib.Roomba_isMainBrushOvercurrent(self.handle) == 0 ? false :true
    
    def isSideBrushOvercurrents(self):
        return self.lib.Roomba_isSideBrushOvercurrent(self.handle) == 0 ? false :true
    
    def dirtDetect(self):
        return self.lib.Roomba_dirtDetect(self.handle)

    def getInfraredCharacterOmni(self):
        return self.lib.Roomba_getInfraredCharacterOmni(slef.handle)
    """
    
if __name__ == '__main__':
    roomba = Roomba(roomba.MODEL_500_SERIES, '\\\\.\\COM16', 115200);
    roomba.setMode(roomba.MODE_SAFE)
    roomba.setLED(roomba.LED_DOCK, 127, 127)
    roomba.setMode(roomba.MODE_DOCK)
