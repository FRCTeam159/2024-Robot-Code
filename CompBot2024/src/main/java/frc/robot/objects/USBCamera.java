// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

import org.opencv.core.Mat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

public class USBCamera extends Thread {
  /** Creates a new SwitchCamera. */
  private UsbCamera IntakeCamera;
  protected static CvSource CameraStream;
  private CvSink UsbCameraSink;
  private static Mat m1;

  public USBCamera(int chnl) {
    IntakeCamera = CameraServer.startAutomaticCapture(chnl);
    IntakeCamera.setResolution(640, 480);
    IntakeCamera.setFPS(25);
    CameraStream = CameraServer.putVideo("USBCamera", 640, 480);
    UsbCameraSink = CameraServer.getVideo(IntakeCamera);
  }

  public void putFrame(CvSource source, Mat m) {
    if (m != null) {
      source.putFrame(m);
    }
  }

  private Mat getUsbCameraFrame() {
    Mat mat = new Mat();
    UsbCameraSink.grabFrame(mat);
    return mat;
  }

  @Override
  public void run() {
    // This method will be called once per scheduler run
    while (true) {
      try {
        Thread.sleep(20);
        m1 = getUsbCameraFrame();
        putFrame(CameraStream, m1);
      } catch (Exception exception) {
        System.out.println("error " + exception);
      }

    }
  }
}