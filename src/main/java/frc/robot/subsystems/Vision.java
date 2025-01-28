package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.photonvision.PhotonCamera;

public class Vision extends SubsystemBase {

  private static PhotonCamera photonCamera = new PhotonCamera(Constants.VisionConstants.visionName);
  private Thread m_driverCamThread;
  private boolean targetVisible = false;
  private double targetYaw = 0;
  private final double visionYawOffset = 0;

  public Vision() {

    m_driverCamThread =
        new Thread(
            () -> {
              UsbCamera driverCamera = CameraServer.startAutomaticCapture();

              driverCamera.setResolution(320, 200);

              CvSink cvSink = CameraServer.getVideo();
              CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 400);

              Mat mat = new Mat();

              while (!Thread.interrupted()) {

                if (cvSink.grabFrame(mat) == 0) {

                  outputStream.notifyError(cvSink.getError());
                  continue;

                }

                Imgproc.rectangle(
                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                outputStream.putFrame(mat);
              }

            });

    m_driverCamThread.setDaemon(true);
    m_driverCamThread.start();

    PhotonCamera.setVersionCheckEnabled(false);
  }


  public static PhotonCamera returnCamera() {
    return photonCamera;
  }

  public boolean isTargetVisible() {
    return targetVisible;
  }

  public double getTargetYaw() {
    return targetYaw;
  }

  public double getYawOffset() {
    return visionYawOffset;
  }

  @Override
  public void periodic() {
    targetVisible = false;
    targetYaw = 0.0;

    var results = photonCamera.getAllUnreadResults();
    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          if (target.getFiducialId() == 7) {
            targetYaw = target.getYaw();

            targetVisible = true;
          }
        }
      }
    }
  }
}