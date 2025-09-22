package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorIO.ElevatorIOInputs;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();
  private ElevatorStates state = ElevatorStates.STOW;

  private ElevatorStates targetLevel = ElevatorStates.STOW;

  public static Elevator instance;

  private final TrapezoidProfile elevatorProfile;

  private TrapezoidProfile.State elevatorGoal;
  private TrapezoidProfile.State elevatorCurrentPoint;

  public static Elevator setInstance(ElevatorIO io) {
    instance = new Elevator(io);
    return instance;
  }

  public Elevator(ElevatorIO io) {
    this.io = io;

    this.elevatorProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1000, 1000));
    this.elevatorCurrentPoint = new TrapezoidProfile.State(getPosition(), 0);

    manualSetPosition(elevatorCurrentPoint.position);
  }

  @Override
  public void periodic() {
    Logger.processInputs("Elevator", (LoggableInputs) inputs);

    Logger.recordOutput("ELevator/Position", inputs.position);
    Logger.recordOutput("Elevator/Velocity", inputs.velocityMetersPerSec);
    Logger.recordOutput("Elevator/Current", inputs.currentAmps);
    Logger.recordOutput("Elevator/AtSetpoint", inputs.isAtSetpoint);
  }

  public void manualSetPosition(double position) {
    io.setPosition(position);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public static Elevator getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Elevator instance not set");
    }
    return instance;
  }

  public void setTargetState(ElevatorStates newState) {
    targetLevel = newState;
  }

  public void setState() {
    io.setPosition(targetLevel.position);
    state = targetLevel;
  }

  public ElevatorStates getTargetState() {
    return targetLevel;
  }

  public double getPosition() {
    return inputs.position;
  }

  public double getVelocity() {
    return inputs.velocityMetersPerSec;
  }

  public double getCurrent() {
    return inputs.currentAmps;
  }

  public ElevatorStates getState() {
    return state;
  }

  public boolean atSetpoint() {
    return inputs.isAtSetpoint;
  }

  public void setState(ElevatorStates newState) {
    state = newState;
  }

  public void updateSim() {
    io.updateSim();
  }
}
