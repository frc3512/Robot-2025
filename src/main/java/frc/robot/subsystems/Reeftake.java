package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.lib.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Reeftake extends ProfiledPIDSubsystem {

  private final TalonFX reefAlgaePivotMotor = 
      new TalonFX(Constants.ReeftakeConstants.pivotMotorID);
  private final TalonFX intakeMotor = 
      new TalonFX(Constants.ReeftakeConstants.intakeMotorID);

  boolean coralIn = false;
  boolean shouldScoreCoral = false;

  public Reeftake() {
    super(
        new ProfiledPIDController(
            Constants.ReeftakeConstants.kP,
            Constants.ReeftakeConstants.kI,
            Constants.ReeftakeConstants.kD,
            Constants.ReeftakeConstants.constraints));
    getController().setTolerance(Constants.ReeftakeConstants.tolerance);

    reefAlgaePivotMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void coralIntake() {
      intakeMotor.set(0.5);
  }

  public void coralStop() {
    intakeMotor.set(0.0);
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    reefAlgaePivotMotor.setVoltage(output);
  }

  @Override
  protected double getMeasurement() {
    return reefAlgaePivotMotor.getPosition().getValueAsDouble();
  }
}
