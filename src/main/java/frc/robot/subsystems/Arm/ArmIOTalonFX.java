package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ArmIOTalonFX implements ArmIO {

  private final TalonFX motor;

  // Encoder attached to Carridge
  private final Canandmag encoder = new Canandmag(2);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public ArmIOTalonFX(int leadID) {

    motor = new TalonFX(leadID);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.SensorToMechanismRatio = 1.0 / 1.0; // 1:1 gearing
    // TODO: TUNE THESE VALUES
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kG = 0.5;
    config.Slot0.kA = 0.0;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void setPosition(double position) {
    motor.setControl(new MotionMagicVelocityVoltage(position));
  }

  @Override
  public double getPosition() {
    return encoder.getAbsPosition();
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
  public boolean atSetpoint(ArmStates state) {
    double tolerance = 0.1;
    double position = encoder.getAbsPosition();
    return Math.abs(position - state.position) <= tolerance;
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.position               = getPosition();
    inputs.velocityMetersPerSec   = getVelocityMetersPerSec();
    inputs.appliedVolts           = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps            = motor.getStatorCurrent().getValueAsDouble();
    inputs.temp                   = motor.getDeviceTemp().getValueAsDouble();
  }
}
