package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static class ElevatorConstants {
    public static final int frontMotorID = 13;
    public static final int backMotorID = 14;

    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double tolerance = 0.002;

    public static final double elevatorGearRatio = 11.0 / 50.0;
    public static final double elevatorDrumRadiusMeters = 0.023;

    public static final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(7, 5);
  }

  public static class ClimberConstants {
    public static final int climbMotor1ID = 11;

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

    public static final int digialInputChannel = 0;

    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double tolerance = 0.002;

    public static final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(7, 5);
  }

  public static class GroundtakeConstants {
    public static final int floorAlgaeRollerMotorID = 15;
    public static final int floorAlgaePivotMotorID = 16;

    public static final int encoderID = 30;

    public static final double kP = 9;
    public static final double kI = 0.0;
    public static final double kD = 0.005;

    public static final double tolerance = 0.005;

    public static final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(5, 3);
  }

  public static class VisionConstants {
    public static final String leftCamera = "Arducam OV9281 3512 left";
    public static final String rightCamera = "Arducam OV9281 3512 right";

    public static final Transform3d robotToCam =
        new Transform3d(
            Units.inchesToMeters(-11.0),
            Units.inchesToMeters(7.0),
            Units.inchesToMeters(16.5),
            new Rotation3d(0.0, Units.degreesToRadians(105.0), Units.degreesToRadians(180.0)));
    public static final Transform3d camToRobot = robotToCam.inverse();

    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

    public static final double visionTurnP = 1; // TUNE THIS VALUE
  }
}
