package frc.lib.command.Complex;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class scoreBarge extends InstantCommand{

    private Intake intake;
    
    public scoreBarge(){
        intake = new Intake();
    }

    @Override
    public void initialize() {
        if (intake.hasAlgae()) {
            Commands.sequence(
                Commands.runOnce(() -> intake.outtake()),
                Commands.waitUntil(() -> !intake.hasAlgae()),
                Commands.runOnce(() -> new reset())
            );
        } else {
            Commands.runOnce(() -> new reset());
        }
    }
    
}