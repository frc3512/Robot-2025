package frc.robot.subsystems.Wrist;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;


public class WristIOTalonFX implements WristIO {

  private final TalonFX motor;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public WristIOTalonFX(int motorID) {

    motor = new TalonFX(motorID);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // TODO: TUNE THESE VALUES
    config.Slot0.kP   = 0.05;
    config.Slot0.kI   = 0.0;
    config.Slot0.kD   = 0.0;
    config.Slot0.kS   = 0.0;
    config.Slot0.kV   = 0.0;
    config.Slot0.kG   = 0.5;
    config.Slot0.kA   = 0.0;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void setPosition(double position) {
    motor.setControl(new MotionMagicVelocityVoltage(position));
  }

  @Override
  public double getPosition() {
    StatusSignal<Angle> rawSignal = motor.getRotorPosition();
    var posSignal = rawSignal.getValueAsDouble();
    return posSignal;
  }

  @Override
  public double getVelocityMetersPerSec() {
    StatusSignal<AngularVelocity> rawSignal = motor.getVelocity();
    var velSignal = rawSignal.getValueAsDouble();
    return velSignal;
  }

  @Override
  public double getCurrent() {
    StatusSignal<Current> rawSignal = motor.getStatorCurrent();
    var currentSignal = rawSignal.getValueAsDouble();
    return currentSignal;
  }

  @Override
  public boolean atSetpoint() {
    double positionError = 
        Math.abs(motor.getClosedLoopError().getValueAsDouble());
    return positionError < 0.1;
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.position               = getPosition();
    inputs.velocityMetersPerSec   = getVelocityMetersPerSec();
    inputs.appliedVolts           = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps            = motor.getStatorCurrent().getValueAsDouble();
    inputs.temp                   = motor.getDeviceTemp().getValueAsDouble();
  }
}
