package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {

  public static class GeneralConstants {
    public static final boolean tuningMode = true;

    public static final Mode simMode = Mode.SIM;
    public static final Mode realMode = Mode.REAL;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    
    public static double LOOP_TIME = 0.02;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
}

  public static class ElevatorConstants {
    public static final int frontMotorID = 13;
    public static final int backMotorID = 14;

    public static final double gearRatio = 4.101;

    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double maxVelocity = 1; // meters per second
    public static final double maxAcceleration = 1; // meters per second squared

    public static final boolean brakeMode = true;

    public static final double forwardSoftLimit = 1.35; // max angle in meters
    public static final double reverseSoftLimit = 0; // min angle in meters

    public static final boolean enableStatorLimit = true;
    public static final int statorCurrentLimit = 40;
    public static final boolean enableSupplyLimit = false;
    public static final double supplyCurrentLimit = 40;

    public static final double drumRadius = 0.0254; // meters

    public static final double minheight = 0;
    public static final double maxheight = 1;

    public static final double l1 = 0.2; 
    public static final double l2 = 0.3;  
    public static final double l3 = 0.5; 
    public static final double l4 = 0.8;

    public static final double stow = 0.05; 
  }

}
