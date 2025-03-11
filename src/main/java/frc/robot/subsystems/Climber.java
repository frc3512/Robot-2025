package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Climber extends ProfiledPIDSubsystem {

  private final TalonFX climbMotor1 = new TalonFX(Constants.ClimberConstants.climbMotor1ID);

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

    climbMotor1.setNeutralMode(NeutralModeValue.Brake);
  }

  public Command setClimber(Double speed) {
    return run(() -> climbMotor1.set(speed));
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    climbMotor1.setVoltage(output);
  }

  @Override
  protected double getMeasurement() {
    return climbMotor1.getPosition().getValueAsDouble();
  }
}
