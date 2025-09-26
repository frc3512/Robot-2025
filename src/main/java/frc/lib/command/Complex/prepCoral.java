package frc.lib.command.Complex;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.command.Arm.setArmState;
import frc.lib.command.Elevator.setElevatorState;
import frc.lib.command.Wrist.setWristState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristStates;

public class prepCoral extends InstantCommand {

    private Intake intake;

    public prepCoral() {
        intake = new Intake();
    }

    @Override
    public void initialize() {
        if (intake.hasCoral()) {
            Commands.sequence(
                Commands.runOnce(() -> new setElevatorState(ElevatorStates.PREP_CORAL)),
                Commands.runOnce(() -> new setArmState(ArmStates.STOW)),
                Commands.runOnce(() -> new setWristState(WristStates.VERTICAL)),
                Commands.waitUntil(() -> 
                        Elevator.getInstance().atSetpoint() && 
                        Arm.getInstance().atSetpoint(ArmStates.STOW) && 
                        Wrist.getInstance().atSetpoint())
            );
        } else {
            Commands.runOnce(() -> new reset());
        }
    }

}
