package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {
    
    public static class ElevatorConstants {
        
        public static final int elevatorMotorRt = 13;
        public static final int elevatorMotorLt = 14;

        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;
        
    }

    public static class ClimberConstants {
        
        public static final int climbMotor1 = 10;
        public static final int climbMotor2 = 11;
        
    }
    
    public static class IntakeConstants {

        public static final int floorAlgaeRollerMotor = 15;
        public static final int floorAlgaePivotMotor = 16;
        public static final int reefAlgaePivotMotor = 17;
        public static final int coralRollerMotor = 18;

        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;

    }

    public static class VisionConstants {

    public static final String leftCamera = "3512 Left";
    public static final String rightCamera = "3512 Right";

    public static final Transform3d robotToCam =
        new Transform3d(
            Units.inchesToMeters(-11.0),
            Units.inchesToMeters(7.0),
            Units.inchesToMeters(16.5),
            new Rotation3d(0.0, Units.degreesToRadians(105.0), Units.degreesToRadians(180.0)));
    public static final Transform3d camToRobot = robotToCam.inverse();

    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

    public static final double visionTurnP = 0.08; // This is used for vision alignment
    public static final double visionDriveP = 0.1; // This is used to drive the robot to the target

    }

}
