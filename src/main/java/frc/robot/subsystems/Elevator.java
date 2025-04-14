package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Elevator extends ProfiledPIDSubsystem {

  private final TalonFX frontMotor = new TalonFX(Constants.ElevatorConstants.frontMotorID);
  private final TalonFX backMotor = new TalonFX(Constants.ElevatorConstants.backMotorID);

  public double gravity = 0.5;

  public String selectedLevel = "stow";

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

    frontMotor.setPosition(0.000);

    backMotor.setControl(new Follower(frontMotor.getDeviceID(), false));

    // Be sure to remove this function when using manual control
    enable();
  }

  public boolean isAtSetpoint() {
    return getController().atSetpoint();
  }

  public Command manualElevator(double speed) {
    return run(
        () -> {
          frontMotor.set(speed);
          backMotor.set(speed);
        });
  }

  public void stow() {
    setClampedGoal(Constants.ElevatorConstants.stowPos);
  }

  public void hp() {
    setClampedGoal(Constants.ElevatorConstants.hpPos);
  }

  public void l1() {
    setClampedGoal(Constants.ElevatorConstants.l1Pos);
  }

  public void l2() {
    setClampedGoal(Constants.ElevatorConstants.l2Pos);
  }

  public void l3() {
    setClampedGoal(Constants.ElevatorConstants.l3Pos);
  }

  public void l4() {
    setClampedGoal(Constants.ElevatorConstants.l4Pos);
  }

  public void a1() {
    setClampedGoal(Constants.ElevatorConstants.a1Pos);
  }

  public void a2() {
    setClampedGoal(Constants.ElevatorConstants.a2Pos);
  }

  public void aStow() {
    setClampedGoal(Constants.ElevatorConstants.aStowPos);
  }

  public Command selectLevel(String level) {
    return Commands.runOnce(() -> selectedLevel = level);
  }

  public void zeroMotor() {
    frontMotor.setPosition(0.000);
  }

  public void setClampedGoal(double goal) {
    setGoal(MathUtil.clamp(goal, 0.5, 47));
  }

  @Override
  public void periodic() {
    super.periodic();

    // Pid Graphing
    DogLog.log(
      "Elevator/Elevator Front Motor Encoder", frontMotor.getPosition().getValueAsDouble());
    DogLog.log(
      "Elevator/Elevator Goal", getController().getSetpoint().position);
    DogLog.log(
      "Elevator/Elevator Voltage", frontMotor.getMotorVoltage().getValueAsDouble());

    // General Info
    DogLog.log(
      "Elevator/Front Motor Temp", frontMotor.getDeviceTemp().getValueAsDouble());
    DogLog.log(
      "Elevator/Back Motor Temp", backMotor.getDeviceTemp().getValueAsDouble());
    DogLog.log(
      "Elevator/Selected Scoring level", selectedLevel);

  }

  @Override
  protected void useOutput(double output, State setpoint) {
    frontMotor.setVoltage(output + gravity);
    backMotor.setVoltage(output + gravity);
  }

  @Override
  protected double getMeasurement() {
    return frontMotor.getPosition().getValueAsDouble();
  }
}
