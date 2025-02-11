package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.lib.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class Climber extends ProfiledPIDSubsystem {

  private final TalonFX climbMotor1 = 
    new TalonFX(Constants.ClimberConstants.climbMotor1ID);
  private final TalonFX climbMotor2 = 
    new TalonFX(Constants.ClimberConstants.climbMotor2ID);

  private boolean canClimbUp = false;
  private boolean wantClimbUp = false;
  private boolean shouldClimbUp = false;

  public Climber() {
    super(
        new ProfiledPIDController(
            Constants.ClimberConstants.kP,
            Constants.ClimberConstants.kI,
            Constants.ClimberConstants.kD,
            Constants.ClimberConstants.constraints));
    getController().setTolerance(Constants.ClimberConstants.tolerance);

    climbMotor1.setNeutralMode(NeutralModeValue.Brake);
    climbMotor2.setNeutralMode(NeutralModeValue.Brake);

    climbMotor2.setControl(new Follower(climbMotor1.getDeviceID(), true));
  }

  public void climbUp() {
    climbMotor1.set(0.8);
    climbMotor2.set(0.8);
  }

  public void climbDown() {
    climbMotor1.set(-0.8);
    climbMotor2.set(-0.8);
  }

  public void climbStop() {
    climbMotor1.set(0);
    climbMotor2.set(0);
  }

  public void setClimbUp() {}

  public void climbLogic() {}

  @Override
  protected void useOutput(double output, State setpoint) {
    climbMotor2.setVoltage(output);
  }

  @Override
  protected double getMeasurement() {
    return climbMotor1.getPosition().getValueAsDouble(); // change if needed
  }
}