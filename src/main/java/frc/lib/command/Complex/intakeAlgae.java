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

public class intakeAlgae extends InstantCommand{
    
    private Intake intake;

    public intakeAlgae() {}

    @Override
    public void initialize() {
        Commands.sequence(
            Commands.runOnce(() -> new setElevatorState(ElevatorStates.INTAKE)),
            Commands.runOnce(() -> new setArmState(ArmStates.INTAKE_ALGAE)),
            Commands.runOnce(() -> new setWristState(WristStates.HORIZONTAL)),

            Commands.waitUntil(() -> 
                    Elevator.getInstance().atSetpoint() && 
                    Arm.getInstance().atSetpoint(ArmStates.INTAKE_ALGAE) && 
                    Wrist.getInstance().atSetpoint()),
                    
            Commands.runOnce(() -> intake.intakeAlgae())
        );
    }

}
