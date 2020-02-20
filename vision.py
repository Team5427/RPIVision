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

import cv2
import numpy
import math
from enum import Enum

"""
--------- TARGET PROCESSING ----------
"""

def process(self, source0):
    """
    Runs the pipeline and sets all outputs to new values.
    """

    #HSL INPUTS BOUNDS
    hue = [0.0, 155.75757575757575]
    sat = [52.74280575539568, 255.0]
    lum = [0.0, 255.0]

    #HSL THRESHOLD STEP
    hslthreshholdinput = source0
    out = cv2.cvtColor(hslthresholdinput, cv2.COLOR_BGR2HLS)
    hslthresholdoutput = cv2.inRange(out, (hue[0], lum[0], sat[0]),  (hue[1], lum[1], sat[1]))
        
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

    min_area = 3000.0
    min_perimeter = 30.0
    min_width = 20.0
    max_width = 1000.0
    min_height = 20.0
    max_height = 1000
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
    return hslthresholdoutput, filtercontoursoutput

class Target:
    """ A target object that identifies if target is real"""

    def __init__(self, pts, mat):
        self.topPoint = None
        self.bottomPoint = None
        self.leftPoint = None
        self.rightPoint = None
        self.uprightDiff = 0
        self.sideDiff = 0
        self.proportion = 0
        self.mat = mat
        self.points = []

        for i in range(len(pts)):
            self.points[i] = pts[i]

        topPt = points[0]
        rightPt = points[0]
        leftPt = points[0]
        bottomPt = points[0]

        for pt in self.points:
            if(pt.x < leftPt.x)
                leftPt = pt
            if(pt.x >  rightPt.x)
                rightPt = pt
            if(pt.y > topPt.y)
                topPt = pt
            if(pt.y < bottomPt.y)
                bottomPt = pt

        self.center.y = (topPt.y+bottomPt.y)/2
        self.center.x = (leftPt.x+rightPt.x)/2

        self.uprightDiff = abs(topPt.y - bottomPt.y)
        self.sideDiff = abs(leftPt.x - rightPt.x)

        self.proportion = self.uprightDiff/self.sideDiff

    def getUprightDiff(self):
        return self.uprightDiff

    def getSideDiff(self):
        return self.sideDiff

    def getTopPt(self):
        return self.topPt

    def getBottomPt(self):
        return self.bottomPt

    def getRightPt(self):
        return self.rightPt

    def getLeftPt(self):
        return self.leftPt

    def getProportion(self):
        return self.proportion

    def getDistanceFromCenter(self):
        return self.center.x - self.mat.width()/2

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

    cvsink = CameraServer.getInstance().getVideo()
    outputstream = CameraServer.getInstance().putVideo("processed", 640, 480)
    # loop forever (put all processing code here)
    while True:
        timestamp, img = cvsink.grabFrame(img)
        output, targetcontours = process(img)
        outputstream.putFrame(output)
        time.sleep(10)


