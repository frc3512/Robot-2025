package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("removal")
public class Climber extends SubsystemBase {
    
    public final TalonFX climbMotor1;
    public final TalonFX climbMotor2;

    public boolean canClimbUp = false;
    public boolean wantClimbUp = false;
    public boolean shouldClimbUp = false;
    //make function
    
    public Climber() {
        
        climbMotor1 = new TalonFX(10);
        climbMotor2 = new TalonFX(11);

        climbMotor1.setNeutralMode(NeutralModeValue.Brake);
        climbMotor2.setNeutralMode(NeutralModeValue.Brake);

        climbMotor1.setInverted(false);
        climbMotor2.setInverted(true);

    }

    public void climbUp() {
        
        climbMotor1.set(0.8);
        climbMotor2.set(0.8);

    }

    public void climbDown() {
        
        climbMotor1.set(-0.8);
        climbMotor2.set(-0.8);

    }

    public void climbStop() {
        
        climbMotor1.set(0);
        climbMotor2.set(0);

    }

    public void setClimbUp(){



    }

    public void climbLogic() {

    }
}