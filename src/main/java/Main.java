/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

import edu.wpi.first.vision.VisionPipeline;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;
import org.opencv.core.Mat;

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
       "switched cameras": [
           {
               "name": <virtual camera name>
               "key": <network table key used for selection>
               // if NT value is a string, it's treated as a name
               // if NT value is a double, it's treated as an integer index
           }
       ]
   }
 */

public final class Main {
  private static String configFile = "/boot/frc.json";

  @SuppressWarnings("MemberName")
  public static class CameraConfig {
    public String name;
    public String path;
    public JsonObject config;
    public JsonElement streamConfig;
  }

  @SuppressWarnings("MemberName")
  public static class SwitchedCameraConfig {
    public String name;
    public String key;
  };

  public static int team;
  public static boolean server;
  public static List<CameraConfig> cameraConfigs = new ArrayList<>();
  public static List<SwitchedCameraConfig> switchedCameraConfigs = new ArrayList<>();
  public static List<VideoSource> cameras = new ArrayList<>();

  private Main() {
  }

  /**
   * Report parse error.
   */
  public static void parseError(String str) {
    System.err.println("config error in '" + configFile + "': " + str);
  }

  /**
   * Read single camera configuration.
   */
  public static boolean readCameraConfig(JsonObject config) {
    CameraConfig cam = new CameraConfig();

    // name
    JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    JsonElement pathElement = config.get("path");
    if (pathElement == null) {
      parseError("camera '" + cam.name + "': could not read path");
      return false;
    }
    cam.path = pathElement.getAsString();

    // stream properties
    cam.streamConfig = config.get("stream");

    cam.config = config;

    cameraConfigs.add(cam);
    return true;
  }

  /**
   * Read single switched camera configuration.
   */
  public static boolean readSwitchedCameraConfig(JsonObject config) {
    SwitchedCameraConfig cam = new SwitchedCameraConfig();

    // name
    JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read switched camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    JsonElement keyElement = config.get("key");
    if (keyElement == null) {
      parseError("switched camera '" + cam.name + "': could not read key");
      return false;
    }
    cam.key = keyElement.getAsString();

    switchedCameraConfigs.add(cam);
    return true;
  }

  /**
   * Read configuration file.
   */
  @SuppressWarnings("PMD.CyclomaticComplexity")
  public static boolean readConfig() {
    // parse file
    JsonElement top;
    try {
      top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
    } catch (IOException ex) {
      System.err.println("could not open '" + configFile + "': " + ex);
      return false;
    }

    // top level must be an object
    if (!top.isJsonObject()) {
      parseError("must be JSON object");
      return false;
    }
    JsonObject obj = top.getAsJsonObject();

    // team number
    JsonElement teamElement = obj.get("team");
    if (teamElement == null) {
      parseError("could not read team number");
      return false;
    }
    team = teamElement.getAsInt();

    // ntmode (optional)
    if (obj.has("ntmode")) {
      String str = obj.get("ntmode").getAsString();
      if ("client".equalsIgnoreCase(str)) {
        server = false;
      } else if ("server".equalsIgnoreCase(str)) {
        server = true;
      } else {
        parseError("could not understand ntmode value '" + str + "'");
      }
    }

    // cameras
    JsonElement camerasElement = obj.get("cameras");
    if (camerasElement == null) {
      parseError("could not read cameras");
      return false;
    }
    JsonArray cameras = camerasElement.getAsJsonArray();
    for (JsonElement camera : cameras) {
      if (!readCameraConfig(camera.getAsJsonObject())) {
        return false;
      }
    }

    if (obj.has("switched cameras")) {
      JsonArray switchedCameras = obj.get("switched cameras").getAsJsonArray();
      for (JsonElement camera : switchedCameras) {
        if (!readSwitchedCameraConfig(camera.getAsJsonObject())) {
          return false;
        }
      }
    }

    return true;
  }

  /**
   * Start running the camera.
   */
  public static VideoSource startCamera(CameraConfig config) {
    System.out.println("Starting camera '" + config.name + "' on " + config.path);
    CameraServer inst = CameraServer.getInstance();
    UsbCamera camera = new UsbCamera(config.name, config.path);
    MjpegServer server = inst.startAutomaticCapture(camera);

    Gson gson = new GsonBuilder().create();

    camera.setConfigJson(gson.toJson(config.config));
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    if (config.streamConfig != null) {
      server.setConfigJson(gson.toJson(config.streamConfig));
    }

    return camera;
  }

  /**
   * Start running the switched camera.
   */
  public static MjpegServer startSwitchedCamera(SwitchedCameraConfig config) {
    System.out.println("Starting switched camera '" + config.name + "' on " + config.key);
    MjpegServer server = CameraServer.getInstance().addSwitchedCamera(config.name);

    NetworkTableInstance.getDefault()
        .getEntry(config.key)
        .addListener(event -> {
              if (event.value.isDouble()) {
                int i = (int) event.value.getDouble();
                if (i >= 0 && i < cameras.size()) {
                  server.setSource(cameras.get(i));
                }
              } else if (event.value.isString()) {
                String str = event.value.getString();
                for (int i = 0; i < cameraConfigs.size(); i++) {
                  if (str.equals(cameraConfigs.get(i).name)) {
                    server.setSource(cameras.get(i));
                    break;
                  }
                }
              }
            },
            EntryListenerFlags.kImmediate | EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    return server;
  }

  /**
   * Example pipeline.
   */
  public static class MyPipeline implements VisionPipeline {
    public static int val;
    private static Mat hslThresholdOutput = new Mat();
	private static ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	private static ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();


    public void process(Mat source0) {
      // Step HSL_Threshold0:
      Mat hslThresholdInput = source0;
      double[] hslThresholdHue = {0.0, 155.75757575757575};
      double[] hslThresholdSaturation = {52.74280575539568, 255.0};
      double[] hslThresholdLuminance = {0.0, 255.0};
      hslThreshold(hslThresholdInput, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, hslThresholdOutput);
  
      // Step Find_Contours0:
      Mat findContoursInput = hslThresholdOutput;
      boolean findContoursExternalOnly = false;
      findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);
  
      // Step Filter_Contours0:
      ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
      double filterContoursMinArea = 3000.0;
      double filterContoursMinPerimeter = 30.0;
      double filterContoursMinWidth = 20.0;
      double filterContoursMaxWidth = 1000;
      double filterContoursMinHeight = 20.0;
      double filterContoursMaxHeight = 1000;
      double[] filterContoursSolidity = {0.0, 100.0};
      double filterContoursMaxVertices = 1000000;
      double filterContoursMinVertices = 0;
      double filterContoursMinRatio = 0;
      double filterContoursMaxRatio = 1000;
      filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
  
    }
  
    /**
     * This method is a generated getter for the output of a HSL_Threshold.
     * @return Mat output from HSL_Threshold.
     */
    public static Mat hslThresholdOutput() {
      return hslThresholdOutput;
    }
  
    /**
     * This method is a generated getter for the output of a Find_Contours.
     * @return ArrayList<MatOfPoint> output from Find_Contours.
     */
    public static ArrayList<MatOfPoint> findContoursOutput() {
      return findContoursOutput;
    }
  
    /**
     * This method is a generated getter for the output of a Filter_Contours.
     * @return ArrayList<MatOfPoint> output from Filter_Contours.
     */
    public static ArrayList<MatOfPoint> filterContoursOutput() {
      return filterContoursOutput;
    }
  
  
    /**
     * Segment an image based on hue, saturation, and luminance ranges.
     *
     * @param input The image on which to perform the HSL threshold.
     * @param hue The min and max hue
     * @param sat The min and max saturation
     * @param lum The min and max luminance
     * @param output The image in which to store the output.
     */
    private static void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum,
      Mat out) {
      Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HLS);
      Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
        new Scalar(hue[1], lum[1], sat[1]), out);
    }
  
    /**
     * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
     * @param input The image on which to perform the Distance Transform.
     * @param type The Transform.
     * @param maskSize the size of the mask.
     * @param output The image in which to store the output.
     */
    private static void findContours(Mat input, boolean externalOnly,
      List<MatOfPoint> contours) {
      Mat hierarchy = new Mat();
      contours.clear();
      int mode;
      if (externalOnly) {
        mode = Imgproc.RETR_EXTERNAL;
      }
      else {
        mode = Imgproc.RETR_LIST;
      }
      int method = Imgproc.CHAIN_APPROX_SIMPLE;
      Imgproc.findContours(input, contours, hierarchy, mode, method);
    }
  
  
    /**
     * Filters out contours that do not meet certain criteria.
     * @param inputContours is the input list of contours
     * @param output is the the output list of contours
     * @param minArea is the minimum area of a contour that will be kept
     * @param minPerimeter is the minimum perimeter of a contour that will be kept
     * @param minWidth minimum width of a contour
     * @param maxWidth maximum width
     * @param minHeight minimum height
     * @param maxHeight maximimum height
     * @param Solidity the minimum and maximum solidity of a contour
     * @param minVertexCount minimum vertex Count of the contours
     * @param maxVertexCount maximum vertex Count
     * @param minRatio minimum ratio of width to height
     * @param maxRatio maximum ratio of width to height
     */
    private static void filterContours(List<MatOfPoint> inputContours, double minArea,
      double minPerimeter, double minWidth, double maxWidth, double minHeight, double
      maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
      minRatio, double maxRatio, List<MatOfPoint> output) {
      final MatOfInt hull = new MatOfInt();
      output.clear();
      //operation
      for (int i = 0; i < inputContours.size(); i++) {
        final MatOfPoint contour = inputContours.get(i);
        final Rect bb = Imgproc.boundingRect(contour);
        if (bb.width < minWidth || bb.width > maxWidth) continue;
        if (bb.height < minHeight || bb.height > maxHeight) continue;
        final double area = Imgproc.contourArea(contour);
        if (area < minArea) continue;
        if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
        Imgproc.convexHull(contour, hull);
        MatOfPoint mopHull = new MatOfPoint();
        mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
        for (int j = 0; j < hull.size().height; j++) {
          int index = (int)hull.get(j, 0)[0];
          double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
          mopHull.put(j, 0, point);
        }
        final double solid = 100 * area / Imgproc.contourArea(mopHull);
        if (solid < solidity[0] || solid > solidity[1]) continue;
        if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
        final double ratio = bb.width / (double)bb.height;
        if (ratio < minRatio || ratio > maxRatio) continue;
        output.add(contour);
      }
    }
  
    public static class Target {

      /**
       * The left and right pieces of retroreflective tape
       */
      /**
       * The average of both halves' centers
       */
     
  
      private static Point[]  points;
  
      private static double topWidth;
      private static Point center = new Point();
      private static double widthRatio = 0;
  
      private static Point topPoint = null;
      private static Point bottomPoint = null;
      private static Point leftPoint = null;
      private static Point rightPoint = null;
      private static double uprightDiff = 0;
      private static double sideDiff = 0;
      private static double proportion = 0;
      private static Mat mat;
      
      public Target(Point[] pts, Mat mat) 
      {
          this.mat = mat;
          points = new Point[pts.length];
          for(int x = 0; x<pts.length;x++)
          {
              points[x] = pts[x];
          }
  
          Point topPoint = points[0];
          Point bottomPoint = points[0];
          Point rightPoint = points[0];
          Point leftPoint = points[0];
      
          for (Point p : points)
          {
              if(p.x < leftPoint.x)
                  leftPoint = p;
              if(p.x > rightPoint.x)
                  rightPoint = p;
              if(p.y > topPoint.y)
                  topPoint = p;
              if(p.y < bottomPoint.y)
                  bottomPoint = p;
  
            
          }
  
          center.y = (topPoint.y+bottomPoint.y)/2;
          center.x = (leftPoint.x + rightPoint.x)/2;
  
          uprightDiff  = Math.abs(topPoint.y-bottomPoint.y);
          sideDiff = Math.abs(leftPoint.x-rightPoint.x);
  
          proportion = uprightDiff/sideDiff;
  
          //TODO FILTER IF THERE ARE MULTIPLE TARGETS
      }
      public static double getWidthRatio()
      {
          return widthRatio;
      }
  
      public static double getUprightDiff()
      {
          return uprightDiff;
      }
      public static double getSideDiff()
      {
          return sideDiff;
      }
      public static Point getTopPoint()
      {
          return topPoint;
      }
      public static Point getBottomPoint()
      {
          return bottomPoint;
      }
      public static Point getRightPoint()
      {
          return rightPoint;
      }
      public static Point getLeftPoint()
      {
          return leftPoint;
      }
  
      public static double getDistanceFromCenter()
      {
          return center.x - mat.width()/2;
      }
      public static boolean isCentered()
      {
          return Math.abs(getDistanceFromCenter())<21;
      }
      public static double getProportion()
      {
          return proportion;
      }
     
      // public double getTapeDist()
      // {
      //     double diffX = right.topLeft.x - left.topLeft.x;
      //     double diffY = right.topLeft.y - left.topLeft.y;
      //     double pixDist = (right.topLeft.y != left.topLeft.y)? Math.sqrt(Math.pow(diffX, 2) + Math.pow(diffY, 2)) : diffX;
      //     return pixDist;
      // }
  
      // public double getXOverZ() {
      //     return (center.x - GraphicsPanel.imageCenterX)/GraphicsPanel.focalLen;
      // }
      // public double getYOverZ() {
      //     return (center.y - GraphicsPanel.imageCenterY)/GraphicsPanel.focalLen;
      // }
      // public double getConstant3() {
      //     return getYOverZ()/getXOverZ();
      // }
      // public double getConstant4() {
      //     return 45; //inches, height of target - height of camera
      // }
      // public double solveForX() {
      //     return getConstant4()/getConstant3();
      // }
      // public double solveForZ() {
      //     return solveForX()/getXOverZ();
      // }
      // public double getHorAngle() {
      //     return Math.atan(solveForX());
      //}
      
  
  }

  /**
   * Main.
   */
  public static void main(String... args) {
    if (args.length > 0) {
      configFile = args[0];
    }

    // read configuration
    if (!readConfig()) {
      return;
    }

    // start NetworkTables
    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    if (server) {
      System.out.println("Setting up NetworkTables server");
      ntinst.startServer();
    } else {
      System.out.println("Setting up NetworkTables client for team " + team);
      ntinst.startClientTeam(team);
    }

    // start cameras
    for (CameraConfig config : cameraConfigs) {
      cameras.add(startCamera(config));
    }

    // start switched cameras
    for (SwitchedCameraConfig config : switchedCameraConfigs) {
      startSwitchedCamera(config);
    }

    // start image processing on camera 0 if present
    if (cameras.size() >= 1) {
      VisionThread visionThread = new VisionThread(cameras.get(0),
              new MyPipeline(), pipeline -> {
        // do something with pipeline results
      });
      /* something like this for GRIP:
      VisionThread visionThread = new VisionThread(cameras.get(0),
              new GripPipeline(), pipeline -> {
        ...
      });
       */
      visionThread.start();
    }

    // loop forever
    for (;;) {
      try {
        Thread.sleep(10000);
      } catch (InterruptedException ex) {
        return;
      }
    }
  }

}}
