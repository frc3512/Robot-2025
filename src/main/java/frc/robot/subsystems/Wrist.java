package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Wrist extends SubsystemBase {
    // Wrist subsystem implementation goes here
    // This could include motor controllers, sensors, and methods to control the wrist's position or speed
    private final TalonFX wristMotor = new TalonFX(Constants.WristConstants.wristMotorID);

    public Wrist() {
        // Initialize motors, sensors, etc.
        wristMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setWristPosition(double position) {
        // Logic to set the wrist to a specific position
    }

    @Override
    public void periodic() {
        // Code that runs periodically, such as updating sensor readings or logging
    }
}
