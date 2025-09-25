package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX leadMotor;
  private final TalonFX followerMotor;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public ElevatorIOTalonFX(int leadID, int followerID) {

    leadMotor = new TalonFX(leadID);
    followerMotor = new TalonFX(followerID);

    followerMotor.setControl(new Follower(leadMotor.getDeviceID(), true));

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.SensorToMechanismRatio = 50 / 11;
  
    // TODO: TUNE THESE VALUES
    // Current PID Used is from other robot
    config.Slot0.kP = 0.75;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kG = 0.0;
    config.Slot0.kA = 0.0;

    leadMotor.getConfigurator().apply(config);
    followerMotor.getConfigurator().apply(config);
  }

  @Override
  public void setVoltage(double volts) {
    leadMotor.setVoltage(volts);
  }

  @Override
  public void setPosition(double position) {
    leadMotor.setControl(new MotionMagicVelocityVoltage(position));
    followerMotor.setControl(new MotionMagicVelocityVoltage(position));
  }

  @Override
  public double getPosition() {
    StatusSignal<Angle> frontSignal = leadMotor.getRotorPosition();
    StatusSignal<Angle> backSignal = followerMotor.getRotorPosition();
    var posSignal = (frontSignal.getValueAsDouble() + backSignal.getValueAsDouble()) / 2.0;
    return posSignal;
  }

  @Override
  public double getVelocityMetersPerSec() {
    StatusSignal<AngularVelocity> fontVel = leadMotor.getVelocity();
    StatusSignal<AngularVelocity> backVel = followerMotor.getVelocity();
    var velSignal = (fontVel.getValueAsDouble() + backVel.getValueAsDouble()) / 2.0;
    return velSignal;
  }

  @Override
  public double getCurrent() {
    return (leadMotor.getStatorCurrent().getValueAsDouble()
            + followerMotor.getStatorCurrent().getValueAsDouble()) / 2.0;
  }

  @Override
  public boolean atSetpoint() {
    double positionError = 
        Math.abs(
                (leadMotor.getClosedLoopError().getValueAsDouble()
                + followerMotor.getClosedLoopError().getValueAsDouble())) / 2.0;
    return positionError < 0.1;
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    leadMotor.getConfigurator().apply(config);
    followerMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.position = getPosition();
    inputs.velocityMetersPerSec = getVelocityMetersPerSec();
    inputs.appliedVolts = leadMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = leadMotor.getStatorCurrent().getValueAsDouble();
    inputs.temp = leadMotor.getDeviceTemp().getValueAsDouble();
  }
}
