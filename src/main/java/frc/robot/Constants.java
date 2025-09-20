package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {

  public static class GeneralConstants {
    public static final boolean tuningMode = true;

    public static final Mode simMode = Mode.SIM;
    public static final Mode realMode = Mode.REAL;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static double LOOP_TIME = 0.02;

    public static double mainLoopFrequency = 50d; // Hz
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
    public static final int leadID = 13;
    public static final int followerID = 14;
  }
}
