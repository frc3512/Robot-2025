package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

import frc.robot.Constants;

@SuppressWarnings("unused")
public class Vision extends SubsystemBase {

  private static PhotonCamera leftCamera = new PhotonCamera(Constants.VisionConstants.leftCamera);
  private static PhotonCamera rightCamera = new PhotonCamera(Constants.VisionConstants.rightCamera);
  
  public PhotonPoseEstimator leftPhotonPoseEstimator;
  public PhotonPoseEstimator rightPhotonPoseEstimator;

  public AprilTagFieldLayout layout;

  private boolean targetVisible = false;
  
  private double targetYaw = 0;
  private double targetRange = 0;

  private final double visionYawOffset = 0;
  private final double visionRangeOffset = 0;

  public Vision() {

    layout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    leftPhotonPoseEstimator =
      new PhotonPoseEstimator(
        layout,
        PoseStrategy.CLOSEST_TO_LAST_POSE,
        Constants.VisionConstants.robotToCam);

    rightPhotonPoseEstimator =
      new PhotonPoseEstimator(
        layout,
        PoseStrategy.CLOSEST_TO_LAST_POSE,
        Constants.VisionConstants.robotToCam);


    PhotonCamera.setVersionCheckEnabled(false);

  }


  public static PhotonCamera returnLeftCamera() {

    return leftCamera;

  }

  public static PhotonCamera returnRightCamera() {

    return rightCamera;

  }

  public boolean isTargetVisible() {

    return targetVisible;

  }

  public double getTargetYaw() {

    return targetYaw;

  }

  public double getTargetRange() {

    return targetRange;

  }

  public double getYawOffset() {

    return visionYawOffset;

  }

  public double getRangeOffset() {

    return visionRangeOffset;

  }

  @Override
  public void periodic() {

    var leftResults = leftCamera.getAllUnreadResults();
    var rightResults = rightCamera.getAllUnreadResults();

    if (!leftResults.isEmpty()) {

      var leftResult = leftResults.get(leftResults.size() - 1);
      var rightResult = rightResults.get(rightResults.size() - 1);

      if (leftResult.hasTargets() && rightResult.hasTargets()) {

        for (var target : leftResult.getTargets()) {

          if (target.getFiducialId() == 18 && target.getPoseAmbiguity() < 0.2) {

            targetYaw = target.getYaw();
            targetRange =
                          PhotonUtils.calculateDistanceToTargetMeters(
                          0.25, // Height of the camera above the floor, in meters
                          0.31, // Height of the april tag we are using, in meters
                          Units.degreesToRadians(0), // Angle of the camera relative to the floor, in degrees
                          Units.degreesToRadians(target.getPitch()));

            targetVisible = true;

          }

        }

      }

    }

    SmartDashboard.putBoolean("PhotonVision/Target Visible", isTargetVisible());
    SmartDashboard.putNumber("PhotonVision/Target Yaw", getTargetYaw());
    SmartDashboard.putNumber("PhotonVision/Target Range", getTargetRange());

  }

}