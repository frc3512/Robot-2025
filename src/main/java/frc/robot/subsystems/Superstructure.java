package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase{
    
    public Superstructure() {

    }

    // * --- Elevator Methods ---

    // * -- Arm Methods --

    // * -- Wrist Methods --

    // * -- Main Methods --

    // TODO:
    // Turn into full sequence once other subsytems are added, make sure it follows the following order:
    // 1. Set wrist to vertical position
    // 2. Set arm to stow position
    // 3. Set elevator to stow position
    // Make it a sequential command group so that it runs in order, with waits between each step if necessary.
    public Command stow() {
        return Commands.sequence();
    }

    // TODO:
    // Make full score sequence once other subsystems are added, make sure it follows the following order:
    // 1. Outtake Coral
    // 2. Retract arm to stow position
    // 3. Retract elevator to stow position
    public Command score() {
        return Commands.sequence();
    }

    // TODO:
    // Make full intake sequence once other subsystems are added, make sure it follows the following order:
    // 1. Lower elevator to ground level
    // 2. Extend arm to ground level
    // 3. Rotate wrist to horizontal position
    public Command intake() {
        return Commands.sequence();
    }
}
