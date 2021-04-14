import json
import time
import sys

from networktables import NetworkTables

import logging

"""
--------- TARGET PROCESSING ----------
"""


def processTarget(source0):
    """
    Runs the pipeline and sets all outputs to new values.
    """

    #HSL INPUTS BOUNDS
    hue = [37, 101.0]
    sat = [179, 255.0]
    val = [71, 255]

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

    min_area = 1.0
    min_perimeter = 1.0
    min_width = 1.0
    max_width = 4000.0
    min_height = 1.0
    max_height = 4000
    solidity = [0, 100]
    max_vertex_count = 100000
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
        self.mat = mat #shape(height, width, 3)
        self.points = []
        self.targetWidth = 3.25
        self.FOV_horizontal = 61
        self.FOV_vertical = 34.3
        self.FOV_pixel = self.mat.shape[1]
        self.Tft = 2.8125
        self.focal_length = self.mat.shape[1]/(2 * math.tan((self.FOV_horizontal/2)))
        self.camHeight = 3
        self.camTilt = 0 #degrees not radians


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
           
        self.Tpixel = self.topRightPt[0] - self.topLeftPt[0] #width of top of target

        self.center = [0,0]
        self.center[1] = ((self.topLeftPt[1]+self.bottomPtVal)/2) + ((self.topRightPt[1]+self.bottomPtVal)/2)/2 # y value of center
        self.center[0] = (self.topLeftPt[0]+self.topRightPt[0])/2 # x value of center

        self.leftValDiff = abs(self.topLeftPt[1] - self.bottomPtVal) # height of left
        self.rightValDiff = abs(self.topRightPt[1] - self.bottomPtVal) # height of right

        self.proportion = self.leftValDiff/self.rightValDiff # proportion of left side:right side
        self.size = ((self.leftValDiff+self.rightValDiff)/2) # average height of target
        

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
    
    def getMatWidthHalf(self):
        return self.mat.shape[1]/2

    def isCentered(self):
        return abs(self.getDistanceFromCenter()) < 4

    def getSize(self):
        return self.size

    #NEEDS TO BE TESTED LATER, DO NOT USE
    def getDistanceFromTarget(self):
        # theta = math.radians(self.FOV_horizontal/2)
        # return self.Tft * self.FOV_pixel/(2*self.Tpixel* math.tan(theta))
        return 1/(math.tan(self.getPitchFromTarget())/6.8125)

    def getYawFromTarget(self):
        f = self.focal_length
        u = self.center[0]
        cx = self.mat.shape[1]/2
        theta = math.atan((u-cx)/f)
        return math.degrees(theta)

    def getPitchFromTarget(self):
        f = self.focal_length
        v = self.center[1]
        cy = self.mat.shape[0]/2
        theta  = math.atan((cy-v)/f) + math.radians(self.camTilt)
        return math.degrees(theta)

    #USE THIS FOR THE MODEL FOR DISTANCE
    def getDistanceFromTargetModel(self):
        x = self.size
        return x


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


logging.basicConfig(level=logging.DEBUG)

NetworkTables.initialize(server = "10.54.27.2")

table = NetworkTables.getTable("SmartDashboard")

# table.putNumber("dTime", 776)
# while True:
#     print("Distance: " , table.getNumber("Distance", 0))

while True:

    """ TARGET """
    timestamp, img = cvsink.grabFrame(img)
    output, filteredPoints = processTarget(img) 
    outputstream.putFrame(output)
    validTargets = []
    biggestTarget = None
    points = None
    target = None

    #finds all valid targets in image
    for currentMat in filteredPoints:
        points = np.reshape(currentMat, (currentMat.shape[0], 2))
        target = Target(points, img)
        validTargets.append(target)

    #finds out if there is a target
    targetExists = table.getEntry("targetExists")
    if(len(validTargets)> 0):
        targetExists.setBoolean(True)
        biggestTarget = validTargets[0]
        for target in validTargets:
            if target.getBiggestSideDifference() > biggestTarget.getBiggestSideDifference():
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

        distanceFromTarget = table.getEntry("distanceFromTarget")
        distanceFromTarget.setDouble(biggestTarget.getDistanceFromTarget())

        yawFromTarget = table.getEntry("yawFromTarget")
        yawFromTarget.setDouble(biggestTarget.getYawFromTarget())

        pitchFromTarget = table.getEntry("pitchFromTarget")
        pitchFromTarget.setDouble(biggestTarget.getPitchFromTarget())

        print(table.getEntry("isTargetCentered").getBoolean(False))
    else:
        targetExists.setBoolean(False)
