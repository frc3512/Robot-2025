package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Reeftake extends SubsystemBase {

  private final TalonFX algaeMotor = new TalonFX(17);
  private final TalonFX intakeMotor = new TalonFX(18);

  public final DigitalInput coralIn =
      new DigitalInput(Constants.ReeftakeConstants.digitalInputChannel);

  public boolean isCoralIn() {
    return coralIn.get();
  }

  public Reeftake() {
    algaeMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void coralIntake() {
    intakeMotor.set(0.175);
  }

  public void algaeIntake() {
    algaeMotor.set(0.3);
  }

  public void algaeOuttake() {
    algaeMotor.set(-0.9);
  }

  public void algaeStop() {
    algaeMotor.set(0);
  }

  public void coralStop() {
    intakeMotor.set(0.0);
  }

  public Command runCoralIntake(Double speed) {
    return run(() -> intakeMotor.set(speed));
  }

  public Command autoIntake() {
    return Commands.sequence(
        Commands.runOnce(() -> coralIntake()),
        Commands.waitUntil(() -> !coralIn.get()),
        Commands.runOnce(() -> coralStop()));
  }

  @Override
  public void periodic() {
    // General Info
    SmartDashboard.putNumber("Reeftake/Motor Temp", intakeMotor.getDeviceTemp().getValueAsDouble());
  }
}
