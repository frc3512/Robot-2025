package frc.lib.command.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;

public class setElevatorState extends InstantCommand {
  private ElevatorStates targetState;

  public setElevatorState(ElevatorStates targetState) {
    this.targetState = targetState;
  }

  @Override
  public void initialize() {
    Elevator.getInstance().setState(targetState);
  }
}
