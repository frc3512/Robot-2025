package frc.lib.command.Complex.AutoScoreCoral;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.command.Arm.setArmState;
import frc.lib.command.Complex.scoreCoral;
import frc.lib.command.Elevator.setElevatorState;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;

public class autoL4 extends InstantCommand{

    public autoL4() {}

    @Override
    public void initialize() {
        Commands.sequence(
            Commands.runOnce(() -> new setElevatorState(ElevatorStates.L4)),
            Commands.runOnce(() -> new setArmState(ArmStates.FRONT_SCORE)),

            Commands.waitUntil(() -> 
                    Elevator.getInstance().atSetpoint() && 
                    Arm.getInstance().atSetpoint(ArmStates.FRONT_SCORE)),
            
            Commands.runOnce(() -> new scoreCoral(ElevatorStates.L4))
        );
    }   
    
}
