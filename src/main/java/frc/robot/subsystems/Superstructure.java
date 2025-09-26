package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.command.Arm.setArmState;
import frc.lib.command.Elevator.setElevatorState;
import frc.lib.command.Wrist.setWristState;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Elevator.ElevatorStates;

public class Superstructure extends SubsystemBase {

  public Superstructure() {}

  // * --- Elevator Methods ---

  public Command L4() {
    return new setElevatorState(ElevatorStates.L4);
  }

  public Command L3() {
    return new setElevatorState(ElevatorStates.L3);
  }

  public Command L2() {
    return new setElevatorState(ElevatorStates.L2);
  }

  public Command L1() {
    return new setElevatorState(ElevatorStates.L1);
  }

  public Command HP() {
    return new setElevatorState(ElevatorStates.HP);
  }

  public Command STOW() {
    return new setElevatorState(ElevatorStates.STOW);
  }

  public Command ALGAE_L2() {
    return new setElevatorState(ElevatorStates.ALGAE_L2);
  }

  public Command ALGAE_L1() {
    return new setElevatorState(ElevatorStates.ALGAE_L1);
  }

  public Command ALGAE_STOW() {
    return new setElevatorState(ElevatorStates.ALGAE_STOW);
  }

  public Command BARGE() {
    return new setElevatorState(ElevatorStates.BARGE);
  }

  // * -- Arm Methods --

  public Command SET_FRONT_SCORE() {
    return new setArmState(ArmStates.FRONT_SCORE);
  }

  public Command SET_REAR_INTAKE() {
    return new setArmState(ArmStates.REAR_INTAKE);
  }

  public Command SET_FRONT_INTAKE() {
    return new setArmState(ArmStates.FRONT_INTAKE);
  }

  public Command SET_STOW() {
    return new setArmState(ArmStates.STOW);
  }

  public Command SET_ALGAE_STOW() {
    return new setArmState(ArmStates.ALGAE_STOW);
  }

  public Command SET_ALGAE_REMOVE() {
    return new setArmState(ArmStates.ALGAE_REMOVE);
  }

  public Command SET_ALGAE_BARGE() {
    return new setArmState(ArmStates.ALGAE_BARGE);
  }

  public Command SET_ALGAE_PROCESS() {
    return new setArmState(ArmStates.ALGAE_PROCESS);
  }

  // * -- Wrist Methods --

  public Command WRIST_VERTICAL() {
    return new setWristState(frc.robot.subsystems.Wrist.WristStates.VERTICAL);
  }

  public Command WRIST_HORIZONTAL() {
    return new setWristState(frc.robot.subsystems.Wrist.WristStates.HORIZONTAL);
  }

  // * -- Complex Methods --
}
