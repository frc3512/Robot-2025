package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Elevator extends ProfiledPIDSubsystem {

  private final TalonFX frontMotor = new TalonFX(Constants.ElevatorConstants.frontMotorID);
  private final TalonFX backMotor = new TalonFX(Constants.ElevatorConstants.backMotorID);

  // * Use only a gravity constant
  private double gravity = 0.5;

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
    backMotor.setPosition(0.000);

    backMotor.setControl(new Follower(frontMotor.getDeviceID(), false));

    enable();
  }

  public Command manualElevator(double speed) {
    return run(
        () -> {
          frontMotor.set(speed);
          backMotor.set(speed);
        });
  }

  public void setClampedGoal(double goal) {
    setGoal(MathUtil.clamp(goal, 0.5, 47));
  }

  public double getPosition() {
    StatusSignal<Angle> frontPos = frontMotor.getRotorPosition();
    StatusSignal<Angle> backPos = backMotor.getRotorPosition();
    return (Math.abs(backPos.getValueAsDouble()) + Math.abs(frontPos.getValueAsDouble())) / 2;
  }

  public boolean atGoal() {
    double posError = 
      Math.abs(getController().getPositionError());

    return posError < 0.1;
  }

  @Override
  public void periodic() {
    super.periodic();

    // Values for PID graphing
    DogLog.log("Elevator/ Elevator Pos", getPosition());
    DogLog.log("Elevator/Elevator Goal", getController().getSetpoint().position);
    DogLog.log("Elevator/Elevator Front Voltage", frontMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Elevator/Elevator Back Voltage", backMotor.getMotorVoltage().getValueAsDouble());

    // General Info
    DogLog.log("Elevator/Front Motor Temp", frontMotor.getDeviceTemp().getValueAsDouble());
    DogLog.log("Elevator/Back Motor Temp", backMotor.getDeviceTemp().getValueAsDouble());
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    frontMotor.setVoltage(output + gravity);
    backMotor.setVoltage(output + gravity);
  }

  @Override
  protected double getMeasurement() {
    StatusSignal<Angle> frontPos = frontMotor.getRotorPosition();
    StatusSignal<Angle> backPos = backMotor.getRotorPosition();
    return (Math.abs(backPos.getValueAsDouble()) + Math.abs(frontPos.getValueAsDouble())) / 2;
  }
}
