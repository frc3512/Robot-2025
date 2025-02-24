package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Elevator extends ProfiledPIDSubsystem {
  private final TalonFX frontMotor =
      new TalonFX(Constants.ElevatorConstants.frontMotorID);
  private final TalonFX backMotor =
      new TalonFX(Constants.ElevatorConstants.backMotorID);
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
    Constants.ElevatorConstants.kS, 
    Constants.ElevatorConstants.kG, 
    Constants.ElevatorConstants.kV);

  boolean bypassStop = false;
  double goal = 0.0;

  public Elevator() {
    super(
        new ProfiledPIDController(
            Constants.ElevatorConstants.kP,
            Constants.ElevatorConstants.kI,
            Constants.ElevatorConstants.kD,
            Constants.ElevatorConstants.constraints));
    getController().setTolerance(Constants.ElevatorConstants.tolerance);

    frontMotor.setNeutralMode(NeutralModeValue.Brake);
    backMotor.setNeutralMode(NeutralModeValue.Brake);

    frontMotor.setPosition(0.000000000000000);

    backMotor.setControl(new Follower(frontMotor.getDeviceID(), false));

    setClampedGoal(0.0);
    enable();
  }

  public void elevatorUp() {
    frontMotor.set(-0.25);
    backMotor.set(0.25);
  }

  public void elevatorDown() {
    frontMotor.set(0.1);
    backMotor.set(-0.1);
  }

  public void elevatorStop() {
    frontMotor.set(0.0);
    backMotor.set(0.0);
  }

  public void stow() {
    setClampedGoal(0.0);
  }

  public void hp() {
    setClampedGoal(7.0);
  }

  public void l1() {
    setClampedGoal(8.7);
  }

  public void l2() {
    setClampedGoal(15.8);
  }

  public void l3() {
    setClampedGoal(26.7);
  }

  public void l4() {
    setClampedGoal(42.0);
  }

  public void a1() {
    setClampedGoal(12.55);
  }

  public void a2() {
    setClampedGoal(25.04);
  }

  public void zeroMotor() {
    frontMotor.setPosition(0.000000);
  }

  public void setClampedGoal(double goal) {
    setGoal(MathUtil.clamp(goal, 0.0, 42.0));
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("Elevator/ElevatorFrontMotorEncoder", frontMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Elevator Goal", getController().getSetpoint().position);

    SmartDashboard.putNumber("Elevator/Elevator Voltage", frontMotor.getMotorVoltage().getValueAsDouble());
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    frontMotor.setVoltage(output + feedforward.calculate(getController().getSetpoint().velocity));
    backMotor.setVoltage(output + feedforward.calculate(getController().getSetpoint().velocity));
  }

  @Override
  protected double getMeasurement() {
    return frontMotor.getPosition().getValueAsDouble();
  }
}
