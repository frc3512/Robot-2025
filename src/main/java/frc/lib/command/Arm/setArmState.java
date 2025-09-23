package frc.lib.command.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmStates;

public class setArmState extends InstantCommand{

    private ArmStates targetState;

    public setArmState(ArmStates targetState) {
        this.targetState = targetState;
    } 

    @Override
    public void initialize() {
        Arm.getInstance().setState(targetState);
    }
    
}
