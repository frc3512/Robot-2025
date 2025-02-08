package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Groundtake extends SubsystemBase{
    
    private final TalonFX floorAlgaeRollerMotor;
    private final TalonFX floorAlgaePivotMotor;

    private Canandmag encoder;

    
  private ProfiledPIDController pivotPID =
      new ProfiledPIDController(
          9, 0, 0, new Constraints(Units.degreesToRadians(700), Units.degreesToRadians(700)));

    public Groundtake() {

        floorAlgaeRollerMotor = new TalonFX(15);
        floorAlgaePivotMotor = new TalonFX(16);

        encoder = new Canandmag(30);

        floorAlgaeRollerMotor.setNeutralMode(NeutralModeValue.Brake);
        floorAlgaePivotMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    public void floorAlgaeIntake() {
    
        floorAlgaeRollerMotor.set(0.65);
        
    }
    
    public void floorAlgaeOuttake() {
    
        floorAlgaeRollerMotor.set(-0.5);
        
    }

    public void floorAlgaeStop() {
    
        floorAlgaeRollerMotor.set(0);
        
    }

}
