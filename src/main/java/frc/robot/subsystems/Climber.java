package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Climber extends ProfiledPIDSubsystem {

  private final TalonFX climbMotor = 
      new TalonFX(Constants.ClimberConstants.climbMotorID);

  public final DigitalInput climberBreak =
      new DigitalInput(Constants.ClimberConstants.digitalInputChannel);

  public Climber() {
    super(
        new ProfiledPIDController(
            Constants.ClimberConstants.kP,
            Constants.ClimberConstants.kI,
            Constants.ClimberConstants.kD,
            Constants.ClimberConstants.constraints));
    getController().setTolerance(Constants.ClimberConstants.tolerance);

    climbMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public Command setClimber(Double speed) {
    return run(() -> climbMotor.set(speed));
  }

  public void periodic() {
    // General Info
    SmartDashboard.putNumber(
      "Climber/Climber Motor Temp", climbMotor.getDeviceTemp().getValueAsDouble());
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    climbMotor.setVoltage(output);
  }

  @Override
  protected double getMeasurement() {
    return climbMotor.getPosition().getValueAsDouble();
  }
}
