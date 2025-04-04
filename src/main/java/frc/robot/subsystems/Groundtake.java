package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.lib.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Groundtake extends ProfiledPIDSubsystem {

  private final TalonFX floorAlgaeRollerMotor =
      new TalonFX(Constants.GroundtakeConstants.floorAlgaeRollerMotorID);
  private final TalonFX floorAlgaePivotMotor =
      new TalonFX(Constants.GroundtakeConstants.floorAlgaePivotMotorID);

  private Canandmag encoder = new Canandmag(Constants.GroundtakeConstants.encoderID);

  public Groundtake() {
    super(
        new ProfiledPIDController(
            Constants.GroundtakeConstants.kP,
            Constants.GroundtakeConstants.kI,
            Constants.GroundtakeConstants.kD,
            Constants.GroundtakeConstants.constraints));
    getController().setTolerance(Constants.GroundtakeConstants.tolerance);

    floorAlgaeRollerMotor.setNeutralMode(NeutralModeValue.Brake);
    floorAlgaePivotMotor.setNeutralMode(NeutralModeValue.Brake);

    enable();
  }

  public void floorAlgaeIntake() {
    floorAlgaeRollerMotor.set(0.65);
  }

  public void keepAlgae() {
    floorAlgaeRollerMotor.set(0.3);
  }

  public void floorAlgaeOuttake() {
    floorAlgaeRollerMotor.set(-0.5);
  }

  public void floorAlgaeStop() {
    floorAlgaeRollerMotor.set(0);
  }

  public void retractPivot() {
    setGoal(Constants.GroundtakeConstants.stowPos);
  }

  public void extendPivot() {
    setGoal(Constants.GroundtakeConstants.extendPivot);
  }

  @Override
  public void periodic() {
    super.periodic();

    // PID Graphing
    DogLog.log(
      "Groundtake/Pivot Motor Position", encoder.getAbsPosition());
    DogLog.log(
      "Groundtake/Pivot Goal", getController().getGoal().position);

    // General Info
    DogLog.log(
      "Groundtake/Roller Temp", floorAlgaeRollerMotor.getDeviceTemp().getValueAsDouble());
    DogLog.log(
      "Groundtake/Pivot Temp", floorAlgaePivotMotor.getDeviceTemp().getValueAsDouble());
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    floorAlgaePivotMotor.setVoltage(output);
  }

  @Override
  protected double getMeasurement() {
    return encoder.getAbsPosition();
  }
}
