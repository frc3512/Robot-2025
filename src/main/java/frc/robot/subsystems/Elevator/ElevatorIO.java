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
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setPosition(double position) {}
  
  public default double getPosition() {return 0;}

  public default void updateSim() {}
  
  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}

  public default TalonFX getFrontMotor() {
    return null;
  }
  public default TalonFX getBackMotor() {
    return null;
  }
}