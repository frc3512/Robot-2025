package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Utility;
import frc.robot.Constants;

// fm up is green
// bm up is red

// fm down is red
// bm down is green

public class Elevator extends SubsystemBase {

  private final TalonFX frontMotor =
      new TalonFX(Constants.ElevatorConstants.frontMotorID);
  private final TalonFX backMotor =
      new TalonFX(Constants.ElevatorConstants.backMotorID);
  private MotionMagicVoltage request = new MotionMagicVoltage(0);

  boolean bypassStop = false;
  double goal = 0.0;

  public Elevator() {
    frontMotor.setNeutralMode(NeutralModeValue.Brake);
    backMotor.setNeutralMode(NeutralModeValue.Brake);

    backMotor.setControl(new Follower(frontMotor.getDeviceID(), true));

    var motorConfig = new TalonFXConfiguration();

    FeedbackConfigs feedbackConfigs = motorConfig.Feedback;
    feedbackConfigs.SensorToMechanismRatio = 50 / 11; // gear ratio

    var slot0Configs = motorConfig.Slot0;
    slot0Configs.kP = 0.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    slot0Configs.kS = 0.0;
    slot0Configs.kV = 0.5;
    slot0Configs.kG = 0.0;

    var motionMagicConfigs = motorConfig.MotionMagic;
    motionMagicConfigs
        .withMotionMagicCruiseVelocity(
            RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
        .withMotionMagicJerk(
            RotationsPerSecondPerSecond.per(Second)
                .of(100)); // Take approximately 0.1 seconds to reach max accel

    frontMotor.getConfigurator().apply(slot0Configs);
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
    setElevatorGoal(0.0);
  }

  public void hp() {
    setElevatorGoal(0.2768);
  }

  public void l1() {
    setElevatorGoal(0.2895);
  }

  public void l2() {
    setElevatorGoal(0.5752);
  }

  public void l3() {
    setElevatorGoal(0.9499);
  }

  public void l4() {
    setElevatorGoal(1.5214);
  }

  public void a1() {
    setElevatorGoal(0.2895);
  }

  public void a2() {
    setElevatorGoal(0.7403);
  }

  public void setElevatorGoal(double targetGoalMeters) {
    goal =
      Utility.metersToRotations(targetGoalMeters, 
        Constants.ElevatorConstants.elevatorDrumRadiusMeters,  
        Constants.ElevatorConstants.elevatorGearRatio);
  }

  @Override
  public void periodic() {
    frontMotor.setControl(request.withPosition(goal).withSlot(0));

    SmartDashboard.putNumber("ElevatorFrontMotorEncoder", 
      Utility.rotationsToMeters(frontMotor.getPosition().getValueAsDouble(), 
        Constants.ElevatorConstants.elevatorDrumRadiusMeters,  
        Constants.ElevatorConstants.elevatorGearRatio));
  }
}
