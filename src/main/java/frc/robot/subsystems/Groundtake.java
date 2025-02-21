package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    floorAlgaeRollerMotor.set(0.65);
  }

  public void keepAlgae() {
    floorAlgaeRollerMotor.set(0.1);
  }

  public void floorAlgaeOuttake() {
    floorAlgaeRollerMotor.set(-0.5);
  }

  public void floorAlgaeStop() {
    floorAlgaeRollerMotor.set(0);
  }

  public void retractPivot() {
    setGoal(0.350);
    enable();
  }

  public void extendPivot() {
    setGoal(0.247);
    enable();
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("Groundtake/Pos", getMeasurement());
    SmartDashboard.putNumber(
        "Groundtake/MotorPos", floorAlgaePivotMotor.getRotorPosition().getValueAsDouble());
    SmartDashboard.putNumber("Groundtake/Goal", getController().getGoal().position);
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
