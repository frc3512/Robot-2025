package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.*;

public class Elevator extends SubsystemBase {

    // Declare motors
    private final TalonFX elevatorMotorRt;
    private final TalonFX elevatorMotorLt;

    private MotionMagicVoltage request = new MotionMagicVoltage(0);

    boolean bypassStop = false;

    private Distance elevatorGoal = Meters.of(0.0);
    
        public Elevator() { 
            
            // Set parameters for elevator motors
            elevatorMotorRt = new TalonFX(Constants.ElevatorConstants.elevatorMotorRt);
            elevatorMotorLt = new TalonFX(Constants.ElevatorConstants.elevatorMotorLt);

            elevatorMotorRt.setControl(new Follower(elevatorMotorLt.getDeviceID(), false));

            elevatorMotorRt.setNeutralMode(NeutralModeValue.Brake);
            elevatorMotorLt.setNeutralMode(NeutralModeValue.Brake);

            elevatorMotorRt.setControl(new DutyCycleOut(0.0));

            var motorConfig = new TalonFXConfiguration();

            FeedbackConfigs feedbackConfigs = motorConfig.Feedback;
            feedbackConfigs.SensorToMechanismRatio = 11 / 50; // Gear ratio

            var slot0Configs = motorConfig.Slot0;
            
            slot0Configs.kP = ElevatorConstants.kP;
            slot0Configs.kI = ElevatorConstants.kI;
            slot0Configs.kD = ElevatorConstants.kD;
            slot0Configs.kS = 0.0;
            slot0Configs.kV = 0.0;

            var motionMagicConfigs = motorConfig.MotionMagic;

            motionMagicConfigs.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) 
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)); 
            
            elevatorMotorRt.getConfigurator().apply(slot0Configs);

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

        public void l1() {

            elevatorGoal = Meters.of(1.0); // 1 meter

        }
           
        @Override
        public void periodic() {
           
            elevatorMotorRt.setControl(request.withPosition(metersToRotations(elevatorGoal)));
            SmartDashboard.putNumber("Elevator/ElevatorPos", elevatorMotorRt.getRotorPosition().getValueAsDouble());
           
        }
}
