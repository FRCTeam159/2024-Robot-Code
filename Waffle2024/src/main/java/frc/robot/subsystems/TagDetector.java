// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import frc.robot.objects.AprilTag;

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

  static AprilTag target_tag = null;
  static public int BEST = -1;

  boolean first = true;

  static Pose2d last_pose = null;
  static int target_id = BEST;

  static boolean first_pose=true;

  public static double min_decision_margin=30; // reject tags less than this

  static double maxtime = 0;
  static double grab_time = 0;
  static double detect_time=0;
  static int count = 0;
  private CvSink UsbCameraSink;
  private Mat mat;
  static int IMAGE_WIDTH = 640;
  static int IMAGE_HEIGHT= 480;
  public double hFOV=40.107;
  public double aspect=((double)IMAGE_WIDTH)/IMAGE_HEIGHT;
  public double vFOV=hFOV/aspect;
  public double cx=IMAGE_WIDTH/2.0;
  public double cy=IMAGE_HEIGHT/2.0;
  public double fx=cx/Math.tan(0.5*Math.toRadians(hFOV));
  public double fy=cy/Math.tan(0.5*Math.toRadians(vFOV));
  static String test_image = "frame_0000.jpg";

  public TagDetector(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;

    intakeCamera = CameraServer.startAutomaticCapture(0); // specs for Gazebo camera
    intakeCamera.setResolution(IMAGE_WIDTH, IMAGE_HEIGHT);
    intakeCamera.setFPS(25);
    UsbCameraSink = CameraServer.getVideo(intakeCamera);

    wpi_detector = new AprilTagDetector();
    wpi_detector.addFamily("tag16h5",0);

    wpi_poseEstConfig = new AprilTagPoseEstimator.Config(TargetMgr.targetSize, fx, fy, cx, cy);
    wpi_pose_estimator = new AprilTagPoseEstimator(wpi_poseEstConfig);

    ouputStream = CameraServer.putVideo("RobotCamera", IMAGE_WIDTH, IMAGE_HEIGHT);
    //test();
  }

  // test tag detection jni using an image file
  public void test() {
    Mat mat = Imgcodecs.imread(test_image);
    AprilTag[] tags = getTags(mat);
    if(tags ==null)
    return;
    for (int i = 0; i < tags.length; i++) {
      tags[i].print();
    }
  }

  // return an array of tag info structures from an image
  private AprilTag[] getTags(Mat mat) {
    AprilTag[] atags=null;
    Mat graymat = new Mat();
    Imgproc.cvtColor(mat, graymat, Imgproc.COLOR_RGB2GRAY);
    AprilTagDetection[] detections = wpi_detector.detect(graymat);

    // reject tags with a poor decision margin or out of expected index range
    List<AprilTagDetection> list=new ArrayList<AprilTagDetection>();
    for (int i = 0; i < detections.length; i++) {
      AprilTagDetection dect=detections[i];
      int id=dect.getId();
      if(id<TargetMgr.minTargetId() || id>TargetMgr.maxTargetId())
        continue;
      if(dect.getDecisionMargin()>min_decision_margin)
        list.add(dect);
    }

    int num_tags = list.size();
    if(num_tags==0)
      return null;
  
    atags = new AprilTag[num_tags];
    for (int i = 0; i < num_tags; i++) {
      AprilTagDetection detection = list.get(i);
      Transform3d pose = wpi_pose_estimator.estimate(detection);
      atags[i] = new AprilTag(detections[i], pose);
    }
    return atags;
  }

  static public void reset() {
    maxtime = 0;
    grab_time = 0;
    count = 0;
    detect_time=0;
    first_pose=true;
  }

  static public void setTargetId(int i) {
    target_id = i;
  }

  static public void setBestTarget() {
    target_id = BEST;
  }

  static public Pose2d getLastPose() {
    return last_pose;
  }

  // project a scaler distance and angle to x and y coordinates
  public static Translation2d project(
      double radius, Rotation2d angle) {
    return new Translation2d(angle.getCos() * radius, -angle.getSin() * radius);
  }

  // return position of tag relative to the camera
  public static Pose3d getTargetPose() {
    if (target_tag == null)
      return null;
    return target_tag.getPose();
  }

  // Calculate the position of a tag with respect to the robot
  public static Translation2d robotToTarget(AprilTag tag, Rotation2d gyroAngle) {
    Pose3d pose = tag.getPose();
    double distance = tag.getDistance(); // distance to camera
    double g=gyroAngle.getRadians();
   
    // convert x,y, coords in camera space to camera angle
    double cam_angle = Math.atan2(pose.getTranslation().getY(), pose.getTranslation().getX());
  
    double theta=g+cam_angle; // add in gyro angle
    double x=distance*Math.cos(theta);
    double y=distance*Math.sin(theta);

    return new Translation2d(x,+y);
  }

  @Override
  public void run() {
    
    while (!Thread.interrupted()) {
      try {
        Thread.sleep(50);
        long endtime = 0;

        long startTime = System.nanoTime();
        mat = new Mat();
        UsbCameraSink.grabFrame(mat);
        endtime = System.nanoTime();
        double duration=(endtime - startTime);
        double total=duration;
        grab_time+=duration;

        startTime = System.nanoTime();
        AprilTag[] tags = null;
        if (m_drivetrain.useTags())
          tags = getTags(mat);
        endtime = System.nanoTime();
        duration=(endtime - startTime);
        total+=duration;

        detect_time +=  duration;
        count++;
        if (count > 1)
          maxtime = total > maxtime ? total : maxtime;
        int ntags=tags==null?0:tags.length;
        
        if(m_drivetrain.useTags()){
          String s = String.format("ntags:%d ave grab:%-2.1f detect:%-2.1f ms", ntags,1e-6*grab_time / count, 1e-6*detect_time/count);
          SmartDashboard.putString("Detect", s);
        }
        
        if (/*!TargetMgr.tagsPresent() ||*/ !m_drivetrain.useTags() || tags==null) {
          //System.out.println("No tags detected");
          TargetMgr.setStartPose(tags);
          ouputStream.putFrame(mat);
          continue;
        }
        target_tag = null;

        Arrays.sort(tags, new SortbyDistance());
        if(!TargetMgr.startPoseSet())
          TargetMgr.setStartPose(tags);
        target_tag=tags[0];
        last_pose = null;
  
        for (int i = 0; i < tags.length; i++) {
          
          AprilTag tag = tags[i];
          if (first)
            tag.print();

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
        }
        first = false;

        ouputStream.putFrame(mat);
      } catch (Exception ex) {
        //System.out.println("exception:" + ex);
      }
    }
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
