package frc.lib.command.Complex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.command.Arm.setArmState;
import frc.lib.command.Elevator.setElevatorState;
import frc.lib.command.Wrist.setWristState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Wrist.WristStates;

public class reset extends InstantCommand{

    private Intake intake;

    public reset() {
        intake = new Intake();
    }

    @Override
    public void initialize() {
        new setElevatorState(ElevatorStates.STOW);
        new setArmState(ArmStates.STOW);
        new setWristState(WristStates.VERTICAL);

        intake.stop();
    }   
}
