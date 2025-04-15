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
  }

  public void floorAlgaeIntake() {
    floorAlgaeRollerMotor.set(0.75);
  }

  public void keepAlgae() {
    floorAlgaeRollerMotor.set(0.65);
  }

  public void floorAlgaeOuttake() {
    floorAlgaeRollerMotor.set(-0.5);
  }

  public void floorAlgaeStop() {
    floorAlgaeRollerMotor.set(0);
  }

  public void retractPivot() {
    setGoal(Constants.GroundtakeConstants.stowPos);
    enable();
  }

  public void extendPivot() {
    setGoal(Constants.GroundtakeConstants.extendPivot);
    enable();
  }

  @Override
  public void periodic() {
    super.periodic();

    // Values for PID graphing
    DogLog.log("Groundtake/Pos", getMeasurement());
    DogLog.log("Groundtake/MotorPos", floorAlgaePivotMotor.getRotorPosition().getValueAsDouble());
    DogLog.log("Groundtake/Goal", getController().getGoal().position);

    // General Info
    DogLog.log("Groundtake/Grountake Pivot Temp", floorAlgaePivotMotor.getDeviceTemp().getValueAsDouble());
    DogLog.log("Groundtake/Grountake Roller Temp", floorAlgaeRollerMotor.getDeviceTemp().getValueAsDouble());
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
