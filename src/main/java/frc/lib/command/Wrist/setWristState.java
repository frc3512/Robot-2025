package frc.lib.command.Wrist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristStates;

public class setWristState extends InstantCommand{

    private WristStates targetState;

    public setWristState(WristStates targetState) {
        this.targetState = targetState;
    } 

    @Override
    public void initialize() {
        Wrist.getInstance().setState(targetState);
    }
    
}
