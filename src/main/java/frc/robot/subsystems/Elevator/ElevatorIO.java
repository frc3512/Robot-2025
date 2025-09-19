
package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.hardware.TalonFX;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double leftPositionRotations = 0.0;
    public double leftVelocityRPM = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double leftTemperature = 0.0;

    public double rightPositionRotations = 0.0;
    public double rightVelocityRPM = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
    public double rightTemperature = 0.0;
    public ElevatorStates elevatorTargetState;

    public ElevatorIOData data =
        new ElevatorIOData(
          false, 
          false, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0,
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0);
  }

  record ElevatorIOData(
      boolean motorConnected,
      boolean followerConnected,
      double positionRad,
      double velocityRadPerSec,
      double appliedVolts,
      double torqueCurrentAmps,
      double supplyCurrentAmps,
      double tempCelsius,
      double followerAppliedVolts,
      double followerTorqueCurrentAmps,
      double followerSupplyCurrentAmps,
      double followerTempCelsius) {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setPosition(double position) {}
  
  public default double getPosition() {return 0;}

  public default void updateSim() {}

  public default void configurePID(double kP, double kI, double kD) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void runPosition(double positionRad, double feedforward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}


  public default TalonFX getRightMotor() {
    return null;
  }
  public default TalonFX getLeftMotor() {
    return null;
  }
}
