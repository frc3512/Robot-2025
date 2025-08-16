package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;

public class Superstructure extends SubsystemBase{
    
    private final Elevator elevator;
    private final LED leds;
    private final Swerve drivetrain;

    public Superstructure() {

        elevator = new Elevator();
        leds = new LED();
        drivetrain = DriveConstants.createDrivetrain();

    }

    public Command setElevtorLevel(double height) {
        return runOnce(() -> elevator.moveToHeightCommand(height));
    }

}
