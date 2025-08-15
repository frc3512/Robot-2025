package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static class VisionConstants {
    public static final String leftCam = "leftCam"; // Previously "ElevatorCam"
    public static final String rightCam = "RightCam"; // Previously "ClimberCam"

    public static final AprilTagFieldLayout tagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final Transform3d leftCameraOffset =
        new Transform3d( // Always measure from center of the robot
            Units.inchesToMeters(6.0), // Forward / Back
            Units.inchesToMeters(-11), // Left / Right
            Units.inchesToMeters(10.5), // Height
            new Rotation3d( 
                Units.degreesToRadians(90),
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(0.0)));

    public static final Transform3d rightCameraOffset =
        new Transform3d( // Always measure from center of the robot
            Units.inchesToMeters(6.0), // Forward / Back
            Units.inchesToMeters(-11), // Left / Right
            Units.inchesToMeters(10.5), // Height
            new Rotation3d(
              Units.degreesToRadians(0.0), 
              Units.degreesToRadians(0.0), 
              Units.degreesToRadians(0.0)));

    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(1, 1, 2);
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.2);
  }

  public static class AutoConstants {
    public static final double xP = 10;
    public static final double xI = 0.0;
    public static final double xD = 0.0;

    public static final double yP = 10;
    public static final double yI = 0.0;
    public static final double yD = 0.0;

    public static final double thetaP = 7.5;
    public static final double thetaI = 0.0;
    public static final double thetaD = 0.0;
  }

  public static class AimingConstants {
    public static final double xP = 7.5;
    public static final double xI = 0.0;
    public static final double xD = 0.0;

    public static final double yP = 7.5;
    public static final double yI = 0.0;
    public static final double yD = 0.0;

    public static final double thetaP = 5;
    public static final double thetaI = 0.0;
    public static final double thetaD = 0.0;

    public static final TrapezoidProfile.Constraints aimingTranslationConstraints =
        new TrapezoidProfile.Constraints(1, 2);
    public static final TrapezoidProfile.Constraints aimingRotationConstraints =
        new TrapezoidProfile.Constraints(Units.rotationsToRadians(1), Units.rotationsToRadians(2));
  }

  // Credit to 6657
  public static class FieldConstants {
    private static Pose2d getRedReefPose(Pose2d reefPose) {
      return new Pose2d(
          reefPose.getTranslation().getX() + 8.565,
          reefPose.getTranslation().getY(),
          reefPose.getRotation());
    }

    public static class ReefSlot {
      public Pose2d middle;
      public Pose2d left;
      public Pose2d right;
      public Pose2d algae;

      ReefSlot(Pose2d middle, Pose2d left, Pose2d right, Pose2d algae) {
        this.middle = middle;
        this.left = left;
        this.right = right;
        this.algae = algae;
      }
    }

    public static enum ReefPoses {
      Reef_1(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
      // Reef_2(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
      // Reef_3(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
      // Reef_4(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
      // Reef_5(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
      // Reef_6(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

      public ReefSlot blue;
      public ReefSlot red;

      // Shift the pose to the robot's left
      public Pose2d getLeftPose(Pose2d pose) {
        return pose.transformBy(new Transform2d(-0.2, -0.16, new Rotation2d()));
      }

      // Shift the pose to the robot's right
      public Pose2d getRightPose(Pose2d pose) {
        return pose.transformBy(new Transform2d(-0.2, 0.16, new Rotation2d()));
      }

      // Shift the pose to the center
      public Pose2d getAlgaePose(Pose2d pose) {
        return pose.transformBy(new Transform2d(-0.2, -0.0, new Rotation2d()));
      }

      ReefPoses(Pose2d pose) {
        this.blue = new ReefSlot(pose, getLeftPose(pose), getRightPose(pose), getAlgaePose(pose));
        this.red =
            new ReefSlot(
                getRedReefPose(pose),
                getRedReefPose(getLeftPose(pose)),
                getRedReefPose(getRightPose(pose)),
                getRedReefPose(getAlgaePose(pose)));
      }
    }
  }
}
