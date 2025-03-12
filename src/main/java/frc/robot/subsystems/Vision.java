package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.PhotonCamera;

public class Vision extends SubsystemBase {

  private static PhotonCamera photonCamera = 
      new PhotonCamera(Constants.VisionConstants.elevatorCam);

  private boolean targetVisible = false;

  private double targetYaw = 0;
  private final double visionYawOffset = 0;

  public Vision() {
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

    SmartDashboard.putBoolean("photonvision/TargetVisible", targetVisible);

    targetVisible = false;
    targetYaw = 0.0;

    var results = photonCamera.getAllUnreadResults();

    if (!results.isEmpty()) {

      var result = results.get(results.size() - 1);

      if (result.hasTargets()) {

        for (var target : result.getTargets()) {

          if (target.getFiducialId() == 18) {

            targetYaw = target.getYaw();

            targetVisible = true;
          }
        }
      }
    }
  }
}
