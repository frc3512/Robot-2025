package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private final DigitalInput climbBeamBreak = new DigitalInput(Constants.ClimberConstants.digitalInputChannel);

  private final TalonFX climbMotor = 
      new TalonFX(Constants.ClimberConstants.climbMotorID);

  public boolean isBeamBroken() {
    return climbBeamBreak.get();
  }

  public Climber() {
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


}
