package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.lib.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Arm extends ProfiledPIDSubsystem {
    
    private final TalonFX armMotor = 
        new TalonFX(Constants.ArmConstants.armMotorID);

    public Arm() {
        super(
        new ProfiledPIDController(
            Constants.ElevatorConstants.kP,
            Constants.ElevatorConstants.kI,
            Constants.ElevatorConstants.kD,
            Constants.ElevatorConstants.constraints));

        armMotor.setNeutralMode(NeutralModeValue.Brake);

        armMotor.setPosition(0.0000000); // Use lots of zeros to ensure precision
    }

    public void setArmPosition(double position) {
        // Logic to set the arm to a specific position
    }

    @Override
    public void periodic() {
        // Code that runs periodically, such as updating sensor readings or logging
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'useOutput'");
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMeasurement'");
    }
    
}
