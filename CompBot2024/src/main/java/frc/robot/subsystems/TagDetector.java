// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import org.opencv.core.Core;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import objects.AprilTag;

public class TagDetector extends Thread {
  static {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
  }

  private UsbCamera intakeCamera;
  public static double maxPoseError = 2;

  protected static CvSource ouputStream;
  protected AprilTagDetector wpi_detector;

  AprilTagPoseEstimator.Config wpi_poseEstConfig;
  AprilTagPoseEstimator wpi_pose_estimator;

  Drivetrain m_drivetrain;

  static boolean m_targeting = false;

  static AprilTag[] tags = null;

  public static double min_decision_margin = 30; // reject tags less than this

  static int count = 0;
  private CvSink UsbCameraSink;
  private Mat mat;
  static int IMAGE_WIDTH = 640;
  static int IMAGE_HEIGHT = 480;
  public double hFOV = 40.107;
  public double aspect = ((double) IMAGE_WIDTH) / IMAGE_HEIGHT;
  public double vFOV = hFOV / aspect;
  public double cx = IMAGE_WIDTH / 2.0;
  public double cy = IMAGE_HEIGHT / 2.0;
  public double fx = cx / Math.tan(0.5 * Math.toRadians(hFOV));
  public double fy = cy / Math.tan(0.5 * Math.toRadians(vFOV));

  public TagDetector(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
  }

  @Override
  public void run() {
    mat = new Mat();
    intakeCamera = CameraServer.startAutomaticCapture(0); // specs for Gazebo camera
    intakeCamera.setResolution(IMAGE_WIDTH, IMAGE_HEIGHT);
    intakeCamera.setFPS(25);
    UsbCameraSink = CameraServer.getVideo(intakeCamera);

    wpi_detector = new AprilTagDetector();
    try{
      wpi_detector.addFamily("tag16h5", 0);
      wpi_detector.addFamily("tag36h11", 0);
      System.out.println("Tag Families loaded");
    } catch (Exception ex){
      System.out.println("TagDetector exception:" + ex);
    }

    wpi_poseEstConfig = new AprilTagPoseEstimator.Config(TargetMgr.targetSize, fx, fy, cx, cy);
    wpi_pose_estimator = new AprilTagPoseEstimator(wpi_poseEstConfig);

    ouputStream = CameraServer.putVideo("RobotCamera", IMAGE_WIDTH, IMAGE_HEIGHT);

    while (!Thread.interrupted()) {
      try {
        Thread.sleep(30);
        long tm = UsbCameraSink.grabFrame(mat);
        if (tm == 0) // bad frame
          continue;

        boolean autoselect = Autonomous.getAutoset();
        boolean showtags = Autonomous.getShowtags();

        tags = null;

        if (m_targeting || showtags) {
          tags = getTags(mat);
          if (tags != null && tags.length > 1) 
            Arrays.sort(tags, new SortbyDistance());
        }

        // set initial starting position and alliance

        if (autoselect && !TargetMgr.startPoseSet()) {
          int alliance = Autonomous.getAlliance();
          int position = Autonomous.getPosition();
          TargetMgr.setTarget(alliance, position);
        }
        if (tags != null)
          showTags(tags, mat);
        ouputStream.putFrame(mat);
      } catch (Exception ex) {
        System.out.println("TagDetector exception:" + ex);
      }
    }
  }

  // targeting methods
  public static void setTargeting(boolean state) {
    System.out.println("SET TARGETING " + state);
    m_targeting = state;
  }

  public static AprilTag[] getTags() {
    return tags;
  }

  public static boolean isTargeting() {
    return m_targeting;
  }

  void showTags(AprilTag[] tags, Mat mat) {
    for (int i = 0; i < tags.length; i++) {
      AprilTag tag = tags[i];

      Point c = tag.center();

      Scalar lns = new Scalar(255.0, 255.0, 0.0);
      Imgproc.line(mat, tag.tl(), tag.tr(), lns, 2);
      Imgproc.line(mat, tag.tr(), tag.br(), lns, 2);
      Imgproc.line(mat, tag.br(), tag.bl(), lns, 2);
      Imgproc.line(mat, tag.bl(), tag.tl(), lns, 2);

      // Imgproc.rectangle(mat, tl, br, new Scalar(255.0, 255.0, 0.0), 2);
      Imgproc.drawMarker(mat, c, new Scalar(0, 0, 255), Imgproc.MARKER_CROSS, 35, 2, 8);
      Point p = new Point(tag.bl().x - 10, tag.bl().y - 10);
      Imgproc.putText(
          mat, // Matrix obj of the image
          "[" + tag.getTagId() + "]", // Text to be added
          p, // point
          Imgproc.FONT_HERSHEY_SIMPLEX, // front face
          1, // front scale
          new Scalar(255, 0, 0), // Scalar object for color
          2 // Thickness
      );
      if(i==TargetMgr.kBestTarget && m_targeting){
        c.y+=TargetMgr.kVertOffset*IMAGE_HEIGHT;
        c.x+=TargetMgr.kHorizOffset*IMAGE_WIDTH;
        Imgproc.drawMarker(mat, c, new Scalar(0, 0, 255), Imgproc.MARKER_CROSS, 35, 2, 8);
      }
    }
  }

  // return an array of tag info structures from an image
  private AprilTag[] getTags(Mat mat) {
    AprilTag[] atags = null;
    Mat graymat = new Mat();
    Imgproc.cvtColor(mat, graymat, Imgproc.COLOR_RGB2GRAY);
    AprilTagDetection[] detections = wpi_detector.detect(graymat);

    // reject tags with a poor decision margin or out of expected index range
    List<AprilTagDetection> list = new ArrayList<AprilTagDetection>();
    for (int i = 0; i < detections.length; i++) {
      AprilTagDetection dect = detections[i];
      int id = dect.getId();
      if (id < TargetMgr.minTargetId() || id > TargetMgr.maxTargetId())
        continue;
      if (dect.getDecisionMargin() > min_decision_margin)
        list.add(dect);
    }

    int num_tags = list.size();
    if (num_tags == 0)
      return null;

    atags = new AprilTag[num_tags];
    for (int i = 0; i < num_tags; i++) {
      AprilTagDetection detection = list.get(i);
      Transform3d pose = wpi_pose_estimator.estimate(detection);
      atags[i] = new AprilTag(detections[i], pose);
    }
    return atags;
  }

  // Helper class extending Comparator interface
  // sort by distance (closest on top)
  class SortbyDistance implements Comparator<AprilTag> {
    public int compare(AprilTag p1, AprilTag p2) {
      double d1 = p1.getDistance();
      double d2 = p2.getDistance();
      if (d1 < d2)
        return -1;
      if (d1 > d2)
        return 1;
      return 0;
    }
  }
}
