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
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static TrapezoidProfile.Constraints constraints = 
      new TrapezoidProfile.Constraints(100, 200);

    public static final double tolerance = 0.01;

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
    public static final double kP = 10;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double tolerance = 0.002;

    public static final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(1000, 1000);

    // ! Needs tuning
    // Reset due to added gear rario in config for better acuracy
    public static final double stow = 0.2;
    public static final double intake = 10.27;
    public static final double prepCoral = 0.0;

    public static final double l1 = 9.05;
    public static final double l2 = 4.09;
    public static final double l3 = 15.56;
    public static final double l4 = 37.39;

    public static final double a1 = 28.75;
    public static final double a2 = 39.12;
    public static final double aStow = 43.6;
    public static final double barge = 0;

  }

  public static class WristConstants {
    // Todo: TUNE PID
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double tolerance = 0.01;

    public static final TrapezoidProfile.Constraints constraints = 
      new TrapezoidProfile.Constraints(50, 50);

    // ! Tune all these
    public static final double horizontal = 0.000;
    public static final double vertical = 0.064;
  }

  public static class VisionConstants {
    public static final String elevatorCam = "ElevatorCam";
    public static final String climberCam = "ClimberCam";

    public static final AprilTagFieldLayout tagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final Transform3d elevatorCamOffset =
        new Transform3d(
            Units.inchesToMeters(11), // Left to right
            Units.inchesToMeters(-6), // Front to back
            Units.inchesToMeters(17), // Bottom to top
            new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(0)));

    public static final Transform3d climberCamOffset =
        new Transform3d(
            Units.inchesToMeters(8),
            Units.inchesToMeters(11),
            Units.inchesToMeters(6.5),
            new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(0)));

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
    public static final double xP = 5;
    public static final double xI = 0.0;
    public static final double xD = 0.0;

    public static final double yP = 5;
    public static final double yI = 0.0;
    public static final double yD = 0.0;

    public static final double thetaP = 2.5;
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
      Reef_1(new Pose2d(3.22, 4.08, Rotation2d.fromDegrees(0))),
      Reef_2(new Pose2d(3.78, 5.09, Rotation2d.fromDegrees(-60))),
      Reef_3(new Pose2d(5.16, 5.16, Rotation2d.fromDegrees(-120))),
      Reef_4(new Pose2d(5.6, 4.03, Rotation2d.fromDegrees(-180))),
      Reef_5(new Pose2d(5.11, 2.91, Rotation2d.fromDegrees(120))),
      Reef_6(new Pose2d(3.73, 2.89, Rotation2d.fromDegrees(60)));

      public ReefSlot blue;
      public ReefSlot red;

      // Shift the pose to the coral left node
      public Pose2d getLeftPose(Pose2d pose) {
        return pose.transformBy(new Transform2d(-0.01, 0.38, new Rotation2d()));
      }

      // Shift the pose to the coral right node
      public Pose2d getRightPose(Pose2d pose) {
        return pose.transformBy(new Transform2d(-0.01, 0, new Rotation2d()));
      }

      // Shift the pose to the algae
      public Pose2d getAlgaePose(Pose2d pose) {
        return pose.transformBy(new Transform2d(-0.067, 0.2, new Rotation2d()));
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
