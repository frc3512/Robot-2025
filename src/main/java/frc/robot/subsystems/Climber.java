package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class Climber extends ProfiledPIDSubsystem {

  private final TalonFX climbMotor1 = new TalonFX(Constants.ClimberConstants.climbMotor);
  private final DigitalInput magSwitch1 = new DigitalInput(2);
  private final DigitalInput magSwitch2 = new DigitalInput(3);

  DigitalInput topLimitSwitch = new DigitalInput(3);
  DigitalInput bottomLimitSwitch = new DigitalInput(2);

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
}
