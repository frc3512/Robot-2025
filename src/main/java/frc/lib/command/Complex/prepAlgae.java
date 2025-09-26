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

public class prepAlgae extends InstantCommand {

    private Intake intake;

    public prepAlgae() {
        intake = new Intake();
    }

    @Override
    public void initialize() {
        if (intake.hasAlgae()) {
            Commands.sequence(
                Commands.runOnce(() -> new setElevatorState(ElevatorStates.ALGAE_STOW)),
                Commands.runOnce(() -> new setArmState(ArmStates.ALGAE_STOW)),
                Commands.runOnce(() -> new setWristState(WristStates.HORIZONTAL)),
                Commands.waitUntil(() -> 
                        Elevator.getInstance().atSetpoint() && 
                        Arm.getInstance().atSetpoint(ArmStates.ALGAE_STOW) && 
                        Wrist.getInstance().atSetpoint())
            );
        } else {
            Commands.runOnce(() -> new reset());
        }
    }

}
