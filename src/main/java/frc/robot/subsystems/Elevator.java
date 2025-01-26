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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorMotorRt;
    private final TalonFX elevatorMotorLt;
    private MotionMagicVoltage request = new MotionMagicVoltage(0);

    boolean bypassStop = false;
    
        public Elevator() {  
            elevatorMotorRt = new TalonFX(Constants.ElevatorConstants.elevatorMotorRt);
            elevatorMotorLt = new TalonFX(Constants.ElevatorConstants.elevatorMotorLt);

            elevatorMotorRt.setControl(new Follower(elevatorMotorLt.getDeviceID(), false));

            elevatorMotorRt.setNeutralMode(NeutralModeValue.Brake);
            elevatorMotorLt.setNeutralMode(NeutralModeValue.Brake);

            elevatorMotorRt.setControl(new DutyCycleOut(0.0));

            var motorConfig = new TalonFXConfiguration();

            FeedbackConfigs feedbackConfigs = motorConfig.Feedback;
            feedbackConfigs.SensorToMechanismRatio = 11 / 50; //gear ratio

            var slot0Configs = motorConfig.Slot0;
            slot0Configs.kP = 1.0;
            slot0Configs.kI = 0.0;
            slot0Configs.kD = 0.0;
            slot0Configs.kS = 0.0;
            slot0Configs.kV = 0.0;

            var motionMagicConfigs = motorConfig.MotionMagic;
            motionMagicConfigs.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second cruise
                .withMotionMagicAcceleration(RotationsPerSecondSquared.of(10)) // Take approximately 0.5 seconds to reach max vel
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)); // Take approximately 0.1 seconds to reach max accel 
            

            elevatorMotorRt.getConfigurator().apply(slot0Configs);
        }

    
        public void elevatorUp() {

            elevatorMotorLt.set(-0.5);
            elevatorMotorRt.set(0.5);
    
        }
    
        public void elevatorDown() {

            elevatorMotorLt.set(0.5);
            elevatorMotorRt.set(-0.5);
    
        }  
    
        public void elevatorStop() {
    
            elevatorMotorRt.set(0);
            elevatorMotorLt.set(0);
    
        }

        public void l1() {
            TrapezoidProfile.State goal = new TrapezoidProfile.State(1.0, 0.0);

            elevatorSetpoint = elevatorConstraints.calculate(0.020, elevatorSetpoint, goal);

            request.Position = elevatorSetpoint.position;
            request.Velocity = elevatorSetpoint.velocity;
        }
    
        @Override
        public void periodic() {
            elevatorMotorRt.setControl(request.withPosition());
            SmartDashboard.putNumber("Elevator/ElevatorPos", elevatorMotorRt.getRotorPosition().getValueAsDouble());
        }
}
