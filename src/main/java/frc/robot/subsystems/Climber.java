package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private final DigitalInput climbBeamBreak =
      new DigitalInput(Constants.ClimberConstants.digitalInputChannel);

  private final TalonFX climbMotor = new TalonFX(Constants.ClimberConstants.climbMotorID);

  public final DigitalInput climberBreak =
      new DigitalInput(Constants.ClimberConstants.digitalInputChannel);

  public Climber() {
    climbMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public Command setClimber(Double speed) {
    return run(() -> climbMotor.set(speed));
  }

  public boolean isBeamBroken() {
    return climbBeamBreak.get();
  }

  public void periodic() {
    // General Info
    DogLog.log("Climber/Climber Motor Temp", climbMotor.getDeviceTemp().getValueAsDouble());
  }
}
