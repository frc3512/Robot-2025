package frc.robot.subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ArmIOSim implements ArmIO {

  DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 0.025, 50 / 11), DCMotor.getKrakenX60(1));
  PIDController simController = new PIDController(0.75, 0, 0);
  double targetPosition = 0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.position = motorSim.getAngularPositionRotations();
    inputs.velocityMetersPerSec = motorSim.getAngularVelocityRPM() / 60 * 2 * Math.PI;
    inputs.appliedVolts = motorSim.getInputVoltage();
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    simController.setPID(kP, kI, kD);
  }

  @Override
  public void setPosition(double position) {
    targetPosition = position;
  }

  @Override
  public void updateSim() {
    motorSim.setInputVoltage(
        simController.calculate(motorSim.getAngularPositionRotations(), targetPosition));
    motorSim.update(0.02);
  }

  public double getPosition() {
    return motorSim.getAngularPositionRotations();
  }

  public double getVelocityMetersPerSec() {
    return motorSim.getAngularVelocityRPM() / 60 * 2 * Math.PI;
  }

  public boolean atSetpoint() {
    return Math.abs(motorSim.getAngularPositionRotations() - targetPosition) < 0.1;
  }

  public void setVoltage(double volts) {
    motorSim.setInputVoltage(volts);
  }

  public double getCurrent() {
    return motorSim.getCurrentDrawAmps();
  }
}
