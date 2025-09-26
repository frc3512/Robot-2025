package frc.lib.command.Complex;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.command.Arm.setArmState;
import frc.lib.command.Elevator.setElevatorState;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;

public class prepBarge extends InstantCommand{

    public prepBarge(){}

    @Override
    public void initialize() {
        Commands.sequence(
            Commands.runOnce(() -> new setElevatorState(ElevatorStates.BARGE)),
            Commands.waitUntil(() -> Elevator.getInstance().atSetpoint()),

            Commands.runOnce(() -> new setArmState(ArmStates.ALGAE_BARGE)),
            Commands.waitUntil(() -> Arm.getInstance().atSetpoint(ArmStates.ALGAE_BARGE))
        );
    }
    
}
