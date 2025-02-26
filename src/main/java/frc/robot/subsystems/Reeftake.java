package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Reeftake extends ProfiledPIDSubsystem {

  private final TalonFX reefAlgaePivotMotor = 
      new TalonFX(Constants.ReeftakeConstants.pivotMotorID);
  private final TalonFX intakeMotor = 
      new TalonFX(Constants.ReeftakeConstants.intakeMotorID);

  public final DigitalInput coralIn = 
      new DigitalInput(Constants.ReeftakeConstants.digitalInputChannel);

  public boolean coralDetected() {
    return coralIn.get();
  }


  boolean shouldScoreCoral = false;
  boolean shouldIntakeCoral = false;
  boolean canInakeCoral = false;


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
      intakeMotor.set(0.2);
  }

  public void algaeOuttake() {
      intakeMotor.set(-0.8);
  }

  public void coralStop() {
    intakeMotor.set(0.0);
  }

  public Command runCoralIntake(Double speed) {
    return run (() -> intakeMotor.set(speed));
  }

  public void setClampedGoal(double goal) {
    setGoal(MathUtil.clamp(goal, 2.0, 4.0));
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    reefAlgaePivotMotor.setVoltage(output);
  }

  @Override
  protected double getMeasurement() {
    return reefAlgaePivotMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Reeftake/Reeftake Motor Position", reefAlgaePivotMotor.getPosition().getValueAsDouble());
  } 
}
