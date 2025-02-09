package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Groundtake extends SubsystemBase {

  private final TalonFX floorAlgaeRollerMotor;
  private final TalonFX floorAlgaePivotMotor;

  private Canandmag encoder;

  private double setpoint;

  private ProfiledPIDController pivotPID =
      new ProfiledPIDController(0.13, 0.01, 0, new TrapezoidProfile.Constraints(0.5, 0.2));

  public Groundtake() {

    floorAlgaeRollerMotor = new TalonFX(15);
    floorAlgaePivotMotor = new TalonFX(16);

    encoder = new Canandmag(30);

    floorAlgaeRollerMotor.setNeutralMode(NeutralModeValue.Brake);
    floorAlgaePivotMotor.setNeutralMode(NeutralModeValue.Brake);

    pivotPID.setTolerance(0.075);
  }

  public void floorAlgaeIntake() {

    floorAlgaeRollerMotor.set(0.65);
  }

  public void floorAlgaeOuttake() {

    floorAlgaeRollerMotor.set(-0.5);
  }

  public void floorAlgaeStop() {

    floorAlgaeRollerMotor.set(0);
  }

  public void retractPivot() {

    setpoint = 0.0;
  }

  public void extendPivot() {

    setpoint = 0.5;
  }

  @Override
  public void periodic() {

    floorAlgaePivotMotor.set(pivotPID.calculate(encoder.getAbsPosition(), setpoint));

    SmartDashboard.putNumber("Groundtake/Pos", encoder.getAbsPosition());
    SmartDashboard.putNumber(
        "Groundtake/MotorPos", floorAlgaePivotMotor.getRotorPosition().getValueAsDouble());

    SmartDashboard.putNumber("Groundtake/Goal", pivotPID.getGoal().position);
  }
}
