package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorIO.ElevatorIOInputs;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();

  private ElevatorStates targetLevel = ElevatorStates.STOW;

  public static Elevator instance;

  private final TrapezoidProfile elevatorProfile;

  private TrapezoidProfile.State elevatorGoal;
  private TrapezoidProfile.State elevatorCurrentPoint;

  public static Elevator getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Elevator instance not set");
    }
    return instance;
  }

  public void setTargetState(ElevatorStates newState) {
    targetLevel = newState;
  }

  public double getPosition() {
    return inputs.position;
  }

  public static Elevator setInstance(ElevatorIO io) {
    instance = new Elevator(io);
    return instance;
  }

  public Elevator(ElevatorIO io) {
    this.io = io;

    this.elevatorProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1000, 1000));
    this.elevatorCurrentPoint = new TrapezoidProfile.State(getPosition(), 0);
  }

  public ElevatorIO getElevatorIO() {
    return io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    elevatorCurrentPoint = 
        elevatorProfile.calculate(Constants.GeneralConstants.LOOP_TIME, elevatorCurrentPoint, elevatorGoal);
        
    manualSetPosition(elevatorCurrentPoint.position);

    Logger.processInputs("Elevator", (LoggableInputs) inputs);
  }

  public void setState(ElevatorStates state) {
    if (state.position < 0) state.position = 0;

    this.elevatorGoal = new TrapezoidProfile.State(targetLevel.position, 0);
    targetLevel = state;
  }

  public ElevatorStates getTargetState() {
    return targetLevel;
  }

  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  public void manualSetPosition(double position) {
    if (position < 0) position = 0;

    io.setPosition(position);
  }

  public boolean isAtState(double tolerance) {
    return MathUtil.isNear(this.inputs.position, targetLevel.position, tolerance);
  }

  public void updateSim() {
    io.updateSim();
  }

}
