package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double position = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temp = 0.0;
    public boolean isAtSetpoint = false;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setPosition(double position) {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void updateSim() {}

  public default double getPosition() {
    return 0;
  }

  public default double getCurrent() {
    return 0;
  }

  public default double getVelocityMetersPerSec() {
    return 0;
  }

  public default boolean atSetpoint() {
    return false;
  }
}
