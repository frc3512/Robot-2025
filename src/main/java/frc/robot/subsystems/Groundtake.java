package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Groundtake extends SubsystemBase{
    
    private final TalonFX floorAlgaeRollerMotor;
    private final TalonFX floorAlgaePivotMotor;

    private Canandmag encoder;
    
    private ProfiledPIDController pivotPID = new ProfiledPIDController(
            0.6,
            0,
            0,
        new TrapezoidProfile.Constraints(1, 1));

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

    public void retractPivot() {

        floorAlgaePivotMotor.set(pivotPID.calculate(encoder.getAbsPosition(), Constants.IntakeConstants.pivotIn));

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber(
            "Groundtake/Pos", encoder.getAbsPosition());

    }

}
