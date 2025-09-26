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

// * Scoring logic for levels l2, l3, and l4 *
// * L1 has its own command due to the different arm position *
public class scoreCoral extends InstantCommand {

    private Intake intake;
    private ElevatorStates targetState;
    
    public scoreCoral(ElevatorStates targetState) {
        intake = new Intake();
        this.targetState = targetState;
    }

    @Override
    public void initialize() {
        Commands.sequence(
            Commands.runOnce(() -> new setElevatorState(targetState)),
            Commands.runOnce(() -> new setArmState(ArmStates.FRONT_SCORE)),
            Commands.runOnce(() -> new setWristState(WristStates.VERTICAL)),
            Commands.waitUntil(() -> Elevator.getInstance().atSetpoint()),
            Commands.waitUntil(() -> Arm.getInstance().atSetpoint(ArmStates.FRONT_SCORE)),
            Commands.waitUntil(() -> Wrist.getInstance().atSetpoint()),
            Commands.runOnce(() -> intake.outtake()),
            Commands.waitUntil(() -> !intake.hasCoral()),
            Commands.runOnce(() -> new setElevatorState(ElevatorStates.STOW)),
            Commands.runOnce(() -> new setArmState(ArmStates.STOW)),
            Commands.runOnce(() -> intake.stop())
        );
    }

}
