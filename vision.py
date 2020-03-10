#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
#----------------------------------------------------------------------------

import json
import time
import sys

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance
import ntcore

# import opencv
import logging
import cv2
import numpy as np
import math
from enum import Enum


"""
--------- TARGET PROCESSING ----------
"""


def processTarget(source0):
    """
    Runs the pipeline and sets all outputs to new values.
    """

    #HSL INPUTS BOUNDS
    hue = [44, 83.0]
    sat = [85, 255.0]
    val = [21, 255]

    #HSL THRESHOLD STEP
    # hslthreshholdinput = source0
    out = cv2.cvtColor(source0, cv2.COLOR_BGR2HLS)
    hslthresholdoutput = cv2.inRange(out, (hue[0], val[0], sat[0]),  (hue[1], val[1], sat[1]))
        
    #FIND CONTOURS STEP
    findcontoursinput = hslthresholdoutput
    external_only = False
    if(external_only):
        mode = cv2.RETR_EXTERNAL
    else:
        mode = cv2.RETR_LIST
    method = cv2.CHAIN_APPROX_SIMPLE
    im2, contours, hierarchy =cv2.findContours(findcontoursinput, mode=mode, method=method)
    findcontoursoutput = contours

    #FILTER CONTOURS STEP
    filtercontourscontours = findcontoursoutput
    input_contours = filtercontourscontours

    min_area = 20.0
    min_perimeter = 0.0
    min_width = 20.0
    max_width = 4000.0
    min_height = 20.0
    max_height = 4000
    solidity = [0, 100]
    max_vertex_count = 1000000
    min_vertex_count = 0
    min_ratio = 0
    max_ratio = 1000

    output = []
    for contour in input_contours:
        x,y,w,h = cv2.boundingRect(contour)
        if (w < min_width or w > max_width):
            continue
        if (h < min_height or h > max_height):
            continue
        area = cv2.contourArea(contour)
        if (area < min_area):
            continue
        if (cv2.arcLength(contour, True) < min_perimeter):
            continue
        hull = cv2.convexHull(contour)
        solid = 100 * area / cv2.contourArea(hull)
        if (solid < solidity[0] or solid > solidity[1]):
            continue
        if (len(contour) < min_vertex_count or len(contour) > max_vertex_count):
            continue
        ratio = (float)(w) / h
        if (ratio < min_ratio or ratio > max_ratio):
            continue
        output.append(contour)

    filtercontoursoutput = output
    return hslthresholdoutput, output


class Target:
    """ A Target object that identifies if target is real"""

    def __init__(self, pts, mat):
        
        self.topLeftPt = None
        self.topRightPt = None
        self.bottomLeftPt = None
        self.bottomRightPt = None
        self.bottomPtVal = 0
        self.leftValDiff = 0
        self.rightValDiff = 0
        self.proportion = 0
        self.size = 0
        self.mat = mat
        self.points = []


        for i in range(pts.shape[0]):
            self.points.append(pts[i])

        self.topLeftPt = points[0]
        self.topRightPt = points[0]
        self.bottomLeftPt = points[0]
        self.bottomRightPt = points[0]

        for pt in self.points:
          
            if(pt[0] < self.topLeftPt[0]):
                self.topLeftPt = pt
            if(pt[0] >  self.topRightPt[0]):
                self.topRightPt = pt
            if(pt[1] > self.bottomPtVal):
                self.bottomPtVal = pt[1]
           

        self.center = [0,0]
        self.center[1] = ((self.topLeftPt[1]+self.bottomPtVal)/2) + ((self.topRightPt[1]+self.bottomPtVal)/2)/2
        self.center[0] = (self.topLeftPt[0]+self.topRightPt[0])/2


        self.leftValDiff = abs(self.topLeftPt[1] - self.bottomPtVal)
        self.rightValDiff = abs(self.topRightPt[1] - self.bottomPtVal)

        self.proportion = self.leftValDiff/self.rightValDiff
        self.size = ((self.leftValDiff+self.rightValDiff)/2)

        

    def getLeftValDiff(self):
        return self.leftValDiff

    def getRightValDiff(self):
        return self.rightValDiff

    def getBiggestSideDifference(self):
        if(self.rightValDiff>self.leftValDiff):
            return self.rightValDiff
        else:
             return self.leftValDiff

    def getTopLeftPt(self):
        return self.topLeftPt

    def getBottomLeftPt(self):
        return self.bottomLeftPt

    def getTopRightPoint(self):
        return self.topRightPt

    def getBottomRightPoint(self):
        return self.bottomRightPt

    def getProportion(self):
        return self.proportion

    def getDistanceFromCenter(self):
        return self.center[0] - self.mat.shape[1]/2
        # return self.center[0] - self.mat.width()/2
    
    def getMatWidthHalf(self):
        return self.mat.shape[1]/2


    def isCentered(self):
        return abs(self.getDistanceFromCenter()) < 4

    def getSize(self):
        return self.size


"""
---------- CAMERA CONFIG -------------
"""

#   JSON format:
#   {
#       "team": <team number>,
#       "ntmode": <"client" or "server", "client" if unspecified>
#       "cameras": [
#           {
#               "name": <camera name>
#               "path": <path, e.g. "/dev/video0">
#               "pixel format": <"MJPEG", "YUYV", etc>   // optional
#               "width": <video mode width>              // optional
#               "height": <video mode height>            // optional
#               "fps": <video mode fps>                  // optional
#               "brightness": <percentage brightness>    // optional
#               "white balance": <"auto", "hold", value> // optional
#               "exposure": <"auto", "hold", value>      // optional
#               "properties": [                          // optional
#                   {
#                       "name": <property name>
#                       "value": <property value>
#                   }
#               ],
#               "stream": {                              // optional
#                   "properties": [
#                       {
#                           "name": <stream property name>
#                           "value": <stream property value>
#                       }
#                   ]
#               }
#           }
#       ]
#       "switched cameras": [
#           {
#               "name": <virtual camera name>
#               "key": <network table key used for selection>
#               // if NT value is a string, it's treated as a name
#               // if NT value is a double, it's treated as an integer index
#           }
#       ]
#   }

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    inst = CameraServer.getInstance()
    camera = UsbCamera(config.name, config.path)
    server = inst.startAutomaticCapture(camera=camera, return_server=True)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.getInstance().addSwitchedCamera(config.name)

    def listener(fromobj, key, value, isNew):
        if isinstance(value, float):
            i = int(value)
            if i >= 0 and i < len(cameras):
              server.setSource(cameras[i])
        elif isinstance(value, str):
            for i in range(len(cameraConfigs)):
                if value == cameraConfigs[i].name:
                    server.setSource(cameras[i])
                    break

    NetworkTablesInstance.getDefault().getEntry(config.key).addListener(
        listener,
        ntcore.constants.NT_NOTIFY_IMMEDIATE |
        ntcore.constants.NT_NOTIFY_NEW |
        ntcore.constants.NT_NOTIFY_UPDATE)

    return server



if __name__ == "__main__":

    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    logging.basicConfig(level = logging.DEBUG)

    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)
    table = ntinst.getTable("vision")

    # start cameras
    for config in cameraConfigs:
        cameras.append(startCamera(config))

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)

    #gets and sets frames onto networktables
    cam1 = CameraServer.getInstance()
    cam2 = CameraServer.getInstance()
    
    cvsink = cam1.getVideo()
    cvsink2 = cam2.getVideo()

    outputstream = CameraServer.getInstance().putVideo("TargetProcessed", 160 , 120)
    #outputstream2 = CameraServer.getInstance().putVideo("BallProcessed", 160 , 120)

    # loop forever (put all processing code here)
    img = np.zeros(shape=(120,160,3), dtype = np.uint8)
  #  img2 = np.zeros(shape=(120,160,3), dtype = np.uint8)

    while True:
        timestamp, img = cvSink.grabFrame(img)
      #  print("AAAAAA {}".format(timestamp))
        output, filteredPoints = processTarget(img)
   #     output2, filteredPoints2 = processTarget(img2)

        ballOrDriver = table.getEntry("ballOrDriver").getBoolean(False)
        outputstream.putFrame(output)
   #     outputstream2.putFrame(output2)

        
    #    outputstream2.putFrame(output2)

        validTargets = []
        biggestTarget = None
        points = None
        target = None
        
        for currentMat in filteredPoints:
            points = np.reshape(currentMat, (currentMat.shape[0], 2))
            target = Target(points, img)
            validTargets.append(target)
        
        targetExists = table.getEntry("targetExists")

        if(len(validTargets)> 0):
            targetExists.setBoolean(True)
            print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
            biggestTarget = validTargets[0]
            for target in validTargets:
                
                if target.getBiggestSideDifference()>biggestTarget.getBiggestSideDifference():
                    biggestTarget = target

            isTargetCentered = table.getEntry("isTargetCentered")
            isTargetCentered.setBoolean(biggestTarget.isCentered())

            targetDistanceFromCenter = table.getEntry("targetDistanceFromCenter")
            targetDistanceFromCenter.setDouble(biggestTarget.getDistanceFromCenter())

            biggestSideDifference = table.getEntry("biggestSideDifference")
            biggestSideDifference.setDouble(biggestTarget.getBiggestSideDifference())

            proportion = table.getEntry("proportion")
            proportion.setDouble(biggestTarget.getProportion())

            size = table.getEntry("size")
            size.setDouble(biggestTarget.getSize())

            print(table.getEntry("isTargetCentered").getBoolean(False))
        else:
            print("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB")
            targetExists.setBoolean(False)
