package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    // Declare motors
    private final TalonFX elevatorMotorRt;
    private final TalonFX elevatorMotorLt;

    private MotionMagicVoltage request = new MotionMagicVoltage(0);

    boolean bypassStop = false;
    
        public Elevator() { 
            
            // Set parameters for elevator motors
            elevatorMotorRt = new TalonFX(Constants.ElevatorConstants.elevatorMotorRt);
            elevatorMotorLt = new TalonFX(Constants.ElevatorConstants.elevatorMotorLt);

            elevatorMotorRt.setControl(new Follower(elevatorMotorLt.getDeviceID(), false));

            elevatorMotorRt.setNeutralMode(NeutralModeValue.Brake);
            elevatorMotorLt.setNeutralMode(NeutralModeValue.Brake);

        }

    
        public void elevatorUp() {

            elevatorMotorLt.set(0.5);
            elevatorMotorRt.set(0.5);

        }

        public void elevatorDown() {

            elevatorMotorLt.set(-0.5);
            elevatorMotorRt.set(-0.5);

        }

        public void elevatorStop() {

            elevatorMotorLt.set(0.0);
            elevatorMotorRt.set(0.0);

        }
    
        @Override
        public void periodic() {

            elevatorMotorRt.setControl(request.withPosition(9));
            SmartDashboard.putNumber("Elevator/ElevatorPos", elevatorMotorRt.getRotorPosition().getValueAsDouble());

        }
}
