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

public class grabAlgaeReef extends InstantCommand{


    private Intake intake;
    private ElevatorStates targetState;

    public grabAlgaeReef(ElevatorStates targetState){

        intake = new Intake();
        this.targetState = targetState;

    }

    @Override
    public void initialize() {
        Commands.sequence(
            Commands.runOnce(() -> new setElevatorState(targetState)),
            Commands.runOnce(() -> new setArmState(ArmStates.ALGAE_REMOVE)),
            Commands.runOnce(() -> new setWristState(WristStates.HORIZONTAL)),
            Commands.waitUntil(() -> Elevator.getInstance().atSetpoint()),
            Commands.waitUntil(() -> Arm.getInstance().atSetpoint(ArmStates.ALGAE_REMOVE)),
            Commands.waitUntil(() -> Wrist.getInstance().atSetpoint()),
            Commands.runOnce(() -> intake.intakeAlgae()),
            Commands.runOnce(() -> new setElevatorState(ElevatorStates.ALGAE_STOW)),
            Commands.runOnce(() -> new setArmState(ArmStates.STOW))
        );
    }
    
}
