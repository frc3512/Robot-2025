package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double position = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temp = 0.0;
    public boolean isAtSetpoint = false;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

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
