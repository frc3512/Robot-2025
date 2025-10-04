package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.States.ElevatorStates;

public class Elevator extends SubsystemBase{

  private TalonFX frontMotor = new TalonFX(Constants.ElevatorConstants.frontMotorID);
  private TalonFX backMotor = new TalonFX(Constants.ElevatorConstants.backMotorID);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(ElevatorStates.STOW.position);

  private double desiredState;

  // * Use only a gravity constant
  private double gravity = 0.5;

  private static double clampHeight(double height) {
    return MathUtil.clamp(height, 0.0, 47);
  }

  public Elevator() {

    frontMotor.setNeutralMode(NeutralModeValue.Brake);
    backMotor.setNeutralMode(NeutralModeValue.Brake);

    frontMotor.setPosition(0.000);

    backMotor.setControl(new Follower(frontMotor.getDeviceID(), false));

    config.Feedback.SensorToMechanismRatio = 50.0 / 11.0;
    config.Slot0.withKP(0.75);
    config.Slot0.withKG(0.5);
    config.Slot0.withGravityType(GravityTypeValue.Elevator_Static);
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
        Math.abs(
                (frontMotor.getClosedLoopError().getValueAsDouble()
                + backMotor.getClosedLoopError().getValueAsDouble())) / 2.0;
    return positionError < 0.1;
  }
  
  public double getPosition() {
    StatusSignal<Angle> frontSignal = frontMotor.getRotorPosition();
    StatusSignal<Angle> backSignal = backMotor.getRotorPosition();
    var posSignal = (frontSignal.getValueAsDouble() + backSignal.getValueAsDouble()) / 2.0;
    return posSignal;
  }

  @Override
  public void periodic() {
    frontMotor.setControl(positionRequest.withPosition(desiredState / 1.8798));


    // Values for PID graphing
    DogLog.log("Elevator/ Elevator Front Motor Encoder", frontMotor.getPosition().getValueAsDouble());
    DogLog.log("Elevator/Elevator Front Voltage", frontMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Elevator/Elevator Rear Voltage", backMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Elevator/ELevator Goal", desiredState);
    DogLog.log("Elevator/Elevator Pos", getPosition() * 1.8798);

    // General Info
    DogLog.log("Elevator/Front Motor Temp", frontMotor.getDeviceTemp().getValueAsDouble());
    DogLog.log("Elevator/Back Motor Temp", backMotor.getDeviceTemp().getValueAsDouble());
  }
}