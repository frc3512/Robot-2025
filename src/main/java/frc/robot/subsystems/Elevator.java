package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.States.ElevatorStates;

public class Elevator extends SubsystemBase{

  // Todo: Tune setScoreHeight lowering amount

  private TalonFX frontMotor = new TalonFX(Constants.ElevatorConstants.frontMotorID);
  private TalonFX backMotor = new TalonFX(Constants.ElevatorConstants.backMotorID);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final PositionVoltage positionRequest = new PositionVoltage(ElevatorStates.STOW.position);

  private double desiredState;

  // * Use only a gravity constant
  private double gravity = 0.4;

  private static double clampHeight(double height) {
    return MathUtil.clamp(height, 0, 56);
  }

  public Elevator() {

    frontMotor.setNeutralMode(NeutralModeValue.Brake);
    backMotor.setNeutralMode(NeutralModeValue.Brake);

    frontMotor.setPosition(0.000000000);

    config.Feedback.SensorToMechanismRatio = Constants.ElevatorConstants.GEAR_RATIO;
    
    config.Slot0.withKP(Constants.ElevatorConstants.kP);
    config.Slot0.withKG(gravity);

    config.Slot0.withGravityType(GravityTypeValue.Elevator_Static);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    frontMotor.getConfigurator().apply(config);
    backMotor.getConfigurator().apply(config);
  }

  public Command manualElevator(double speed) {
    return run(
        () -> {
          frontMotor.set(speed);
          backMotor.set(speed);
        });
  }

  public void setClampedGoal(ElevatorStates goal) {
    desiredState = clampHeight(goal.position);
  }

  public boolean atSetpoint() {
    double positionError = 
        Math.abs((frontMotor.getClosedLoopError().getValueAsDouble()));
    return positionError * 
      Constants.ElevatorConstants.PULLEY_CIRCUMFERENCE < 1; // 1 inch of error, make smaller if needed
  }

  @Override
  public void periodic() {
    // Values for PID graphing
    DogLog.log("Elevator/Elevator Front Voltage", frontMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Elevator/Elevator Rear Voltage", backMotor.getMotorVoltage().getValueAsDouble());

    DogLog.log("Elevator/ELevator Goal", desiredState);
    DogLog.log("Elevator/At Goal", atSetpoint());
    DogLog.log("Elevator/Elevator Pos", 
      frontMotor.getPosition().getValueAsDouble()* 
        Constants.ElevatorConstants.PULLEY_CIRCUMFERENCE);

    // General Info
    DogLog.log("Elevator/Front Motor Temp", frontMotor.getDeviceTemp().getValueAsDouble());
    DogLog.log("Elevator/Back Motor Temp", backMotor.getDeviceTemp().getValueAsDouble());

    frontMotor.setControl(
      positionRequest.withPosition(
        desiredState / Constants.ElevatorConstants.PULLEY_CIRCUMFERENCE));

    backMotor.setControl(
      new Follower(
        frontMotor.getDeviceID(), true));
  }
}