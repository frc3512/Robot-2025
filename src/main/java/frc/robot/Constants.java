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
  public static class GeneralConstants {}

  public static class ArmConstants {
    // Todo: TUNE PID
    public static final double kP = 30;
    public static final double kI = 0.0;
    public static final double kD = 0;
    public static final double kA = 25;

    public static final double GEAR_RATIO = 23.0 * (45.0 / 12.0);

    // ! Tune all these and add as needed
    public static final double stow = 0.0;
    public static final double prepScore = 0.472;
    public static final double intake = 0.003;
    public static final double algae = 0.65;
    public static final double score = 0.509;
    public static final double barge = 0.47;
    public static final double processor = 0.0;
    public static final double trough = 0.59;
  }

  public static class ElevatorConstants {
    public static final int frontMotorID = 13;
    public static final int backMotorID = 14;

    // ? Check if PID needs ajustments first
    public static final double kP = 1.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double PULLEY_CIRCUMFERENCE = 1.8798 * Math.PI;
    public static final double GEAR_RATIO = 50.0 / 11.0;
  }

  public static class WristConstants {
    // Todo: TUNE PID
    public static final double kP = 35;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double GEAR_RATIO = 10.178;
    
    public static final int motorID = 16;
  }

  public static class VisionConstants {
    public static final String leftCam = "LeftCam"; // Previously "ElevatorCam"
    public static final String rightCam = "RightCam"; // Previously "ClimberCam"

    public static final AprilTagFieldLayout tagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final Transform3d leftCameraOffset =
        new Transform3d( // Always measure from center of the robot
            Units.inchesToMeters(6.0), // Forward / Back
            Units.inchesToMeters(11), // Left / Right
            Units.inchesToMeters(10.5), // Height
            new Rotation3d( 
                Units.degreesToRadians(0.0),
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
    public static final double xP = 10;
    public static final double xI = 0.0;
    public static final double xD = 0.0;

    public static final double yP = 10;
    public static final double yI = 0.0;
    public static final double yD = 0.0;

    public static final double thetaP = 7.5;
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
      Reef_1(new Pose2d(3.17, 4.01, Rotation2d.fromDegrees(0))),
      Reef_2(new Pose2d(3.78, 5.09, Rotation2d.fromDegrees(-60))),
      Reef_3(new Pose2d(5.16, 5.16, Rotation2d.fromDegrees(-120))),
      Reef_4(new Pose2d(5.6, 4.03, Rotation2d.fromDegrees(-180))),
      Reef_5(new Pose2d(5.11, 2.91, Rotation2d.fromDegrees(120))),
      Reef_6(new Pose2d(3.84, 2.91, Rotation2d.fromDegrees(60)));

      public ReefSlot blue;
      public ReefSlot red;

      // Shift the pose to the robot's left
      public Pose2d getLeftPose(Pose2d pose) {
        return pose.transformBy(new Transform2d(0.0, -0.16, new Rotation2d()));
      }

      // Shift the pose to the robot's right
      public Pose2d getRightPose(Pose2d pose) {
        return pose.transformBy(new Transform2d(-0.18, -0.16, new Rotation2d()));
      }

      // Shift the pose to the center
      public Pose2d getAlgaePose(Pose2d pose) {
        return pose.transformBy(new Transform2d(-0.09, -0.1, new Rotation2d()));
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
