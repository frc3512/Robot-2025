package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private final TalonFX climbMotor1 = new TalonFX(Constants.ClimberConstants.climbMotor1ID);

  DigitalInput topLimitSwitch = new DigitalInput(3);
  DigitalInput bottomLimitSwitch = new DigitalInput(2);
  DigitalInput climbBeamBreak = new DigitalInput(1);

  public Climber() {
    climbMotor1.setNeutralMode(NeutralModeValue.Brake);
  }

  public Command setClimber(double speed) {
    return run(() -> climbMotor1.set(speed));
  }

  public boolean isBeamBroken(){
    return climbBeamBreak.get();
  }
}
