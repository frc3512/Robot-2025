package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    private final TalonFX floorAlgaeRollerMotor;
    private final TalonFX floorAlgaePivotMotor;
    private final TalonFX reefAlgaePivotMotor;
    private final TalonFX intakeMotor;

    private final DigitalInput hasCoral;

    boolean coralIn = false;
    boolean shouldScoreCoral = false;

    public Intake() {

        // Floor algae
        floorAlgaeRollerMotor = new TalonFX(15);
        floorAlgaePivotMotor = new TalonFX(16);

        floorAlgaeRollerMotor.setNeutralMode(NeutralModeValue.Brake);
        floorAlgaePivotMotor.setNeutralMode(NeutralModeValue.Brake);
        
        //Reef algae
        reefAlgaePivotMotor = new TalonFX(17);

        reefAlgaePivotMotor.setNeutralMode(NeutralModeValue.Brake);

        //Intake
        intakeMotor = new TalonFX(18);

        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        //Beam Break
        hasCoral = new DigitalInput(0);

    }

    public void floorAlgaeIntake() {
    
        floorAlgaeRollerMotor.set(0.5);
        
    }
    
    public void floorAlgaeOuttake() {
    
        floorAlgaeRollerMotor.set(-0.5);
        
    }

    public void floorAlgaeStop() {
    
        floorAlgaeRollerMotor.set(0);
        
    }

    public void coralLogic() {

        if (hasCoral.get()) {

            coralIn = true;
            
        } else {
            
            coralIn = false;
            
        }

    }

    public void coralIntake() {

        if (!coralIn) {

            intakeMotor.set(0.5);

        } else {

            intakeMotor.set(0);

        }

    }

    public void coralShoot() {
        
        intakeMotor.set(0.8);
        //check if coral is in before shooting
        //something like this
        /*
        if (coralIn) {

            intakeMotor.set(0.8);

        } else {

            intakeMotor.set(0);

        }
        */
    
    }

    public void reefAlgaeIntake() {

        intakeMotor.set(0.5);

    }

    public void reefAlgaeOuttake() {

        intakeMotor.set(-0.5);

    }
    
    public void reefAlgaeStop() {

        intakeMotor.set(0);

    }

}