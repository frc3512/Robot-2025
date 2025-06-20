package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private final TalonFX intakeMotor = 
        new TalonFX(Constants.IntakeConstants.intakeMotorID);

    public Intake() {
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    
    public Command setIntakeSpeed(double speed) {
        return run(() -> intakeMotor.set(speed));
    }
}