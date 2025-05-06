package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private final DigitalInput climbBeamBreak =
      new DigitalInput(Constants.ClimberConstants.beamBreak);
  private final DigitalInput climberTopSwitch =
      new DigitalInput(Constants.ClimberConstants.climberTopSwitch);
  private final DigitalInput climberBottomSwitch =
      new DigitalInput(Constants.ClimberConstants.climberBottomSwitch);

  private final TalonFX climbMotor = new TalonFX(Constants.ClimberConstants.climbMotorID);

  public Climber() {
    climbMotor.setNeutralMode(NeutralModeValue.Brake);
  }


  public boolean isBeamBroken() {
    return climbBeamBreak.get();
  }

  public boolean climberAtTop() {
    return climberTopSwitch.get();
  }

  public boolean climberAtBottom() {
    return climberBottomSwitch.get();
  }

  public Command setClimber(Double speed) {
    return run(() -> climbMotor.set(speed));
  }

  public Command autoClimb() {
    return Commands.sequence(
      extendClimber(),
      Commands.waitUntil(() -> isBeamBroken()),
      Commands.waitSeconds(1),
      retractClimber()
    );
  }

  public Command retractClimber() {
    return Commands.sequence(
      Commands.runOnce(() -> setClimber(-0.8)),
      Commands.waitUntil(() -> !climberAtBottom()),
      Commands.runOnce(() -> setClimber(0.0))
    );
  }

  public Command extendClimber() {
    return Commands.sequence(
      Commands.runOnce(() -> setClimber(0.8)),
      Commands.waitUntil(() -> !climberAtTop()),
      Commands.runOnce(() -> setClimber(0.0))
    );
  }

  public void periodic() {
    // Cimber Data
    DogLog.log("Climber/Climber Beam Break", isBeamBroken());
    DogLog.log("Climber/Climber At Top", climberAtTop());
    DogLog.log("Climber/Climber At Bottom", climberAtBottom());

    // General Info
    DogLog.log("Climber/Climber Motor Temp", climbMotor.getDeviceTemp().getValueAsDouble());
  }
}
