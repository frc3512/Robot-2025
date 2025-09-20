package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.command.Elevator.setElevatorState;
import frc.robot.subsystems.Elevator.ElevatorStates;

public class Superstructure extends SubsystemBase{
    
    public Superstructure() {

    }

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

    // * -- Wrist Methods --

    // * -- Complex Methods --
}
