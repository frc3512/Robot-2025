package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static class ElevatorConstants {
    public static final int frontMotorID = 13;
    public static final int backMotorID = 14;

    public static final double kP = 0.75;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double tolerance = 0.002;
    public static final double scoringTolerance = 0.1;

    public static final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(1000, 1000);

    public static final double stowPos = 1.0;
    public static final double hpPos = 5.0;
    public static final double l1Pos = 8.7;
    public static final double l2Pos = 14.2;
    public static final double l3Pos = 26;
    public static final double l4Pos = 46;
    public static final double a1Pos = 9.5;
    public static final double a2Pos = 21.84;
    public static final double aStowPos = 14.5;

  }

  public static class ClimberConstants {
    public static final int climbMotorID = 11;
    public static final int digitalInputChannel = 1;

    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double tolerance = 0.002;

    public static final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(7, 5);
  }

  public static class ReeftakeConstants {
    public static final int pivotMotorID = 17;
    public static final int intakeMotorID = 18;

    public static final int digitalInputChannel = 0;

    public static final double kP = 1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.1;
    public static final double kG = 0.2;
    public static final double kS = 0.0;

    public static final double tolerance = 0.002;

    public static final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(7, 5);

    public static final double extendPivot = 1.5;
    public static final double prossecer = 0.6;
    public static final double retractPivot = 0.0;
  }

  public static class GroundtakeConstants {
    public static final int floorAlgaeRollerMotorID = 15;
    public static final int floorAlgaePivotMotorID = 16;

    public static final int encoderID = 30;

    public static final double kP = 9;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double tolerance = 0.02;

    public static final double stowPos = 0.297;
    public static final double extendPivot = 0.135;

    public static final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(5, 3);
  }

  public static class VisionConstants {
    public static final String elevatorCam = "Arducam OV9281 3512 left";
    public static final String climberCam = "Arducam OV9281 3512 right";

    public static final Transform3d elevatorCamOffset =
        new Transform3d(
            Units.inchesToMeters(10.0),
            Units.inchesToMeters(6.0),
            Units.inchesToMeters(20),
            new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(270)));

    public static final Transform3d climberCamOffset =
    new Transform3d(
        Units.inchesToMeters(-10),
        Units.inchesToMeters(6.0),
        Units.inchesToMeters(8),
        new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(90)));

    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(2, 2, 4);
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final Constraints autoAimTranslationConstraints = 
        new Constraints(2, 5);
    public static final Constraints autoAimRotationConstraints =
        new Constraints(Units.rotationsToRadians(3), Units.rotationsToRadians(10));

    public static final ProfiledPIDController xController_Position =
        new ProfiledPIDController(8, 0, 0, autoAimTranslationConstraints);
    public static final ProfiledPIDController yController_Position =
        new ProfiledPIDController(8, 0, 0, autoAimTranslationConstraints);
    public static final ProfiledPIDController thetaController_Position =
        new ProfiledPIDController(5, 0, 0, autoAimRotationConstraints);
  }
}
