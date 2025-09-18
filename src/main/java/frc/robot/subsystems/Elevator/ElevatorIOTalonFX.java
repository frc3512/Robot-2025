package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
=
public class ElevatorIOTalonFX implements ElevatorIO {

    private final TalonFX frontMotor = new TalonFX(13);
    private final TalonFX backMotor = new TalonFX(14);
    private double feedForward = 0.6;
    private Elevator elevator;

    private TalonFXConfiguration frontConfig;

    private TalonFXConfiguration backConfig;

    public ElevatorIOTalonFX() {
        frontConfig = new SparkMaxConfig();
        frontConfig.inverted(false);
        frontConfig.idleMode(IdleMode.kBrake);
        frontConfig.signals.primaryEncoderPositionAlwaysOn(true);
        frontConfig.signals.primaryEncoderPositionPeriodMs(10);
        frontConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        frontConfig.closedLoop.maxMotion.allowedClosedLoopError(0);
        frontConfig.closedLoop.positionWrappingEnabled(false);
        frontConfig.voltageCompensation(10);
        frontConfig.smartCurrentLimit(60, 60);
        frontConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.15, 0.000, 0);
        frontConfig.closedLoop.iZone(5);
        frontMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        frontMotor.setPeriodicFrameTimeout(30);
        frontMotor.setCANTimeout(30);
        frontMotor.setCANMaxRetries(5);
        frontMotor.getEncoder().setPosition(0);

        backConfig = new SparkMaxConfig();
        backConfig.inverted(false);
        backConfig.idleMode(IdleMode.kBrake);
        backConfig.signals.primaryEncoderPositionAlwaysOn(true);
        backConfig.signals.primaryEncoderPositionPeriodMs(10);
        backConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        backConfig.closedLoop.maxMotion.allowedClosedLoopError(0);
        backConfig.closedLoop.positionWrappingEnabled(false);
        backConfig.voltageCompensation(10);
        backConfig.smartCurrentLimit(60, 60);
        backConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.15, 0.000, 0);
        backConfig.closedLoop.iZone(5);
        backMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        backMotor.setPeriodicFrameTimeout(30);
        backMotor.setCANTimeout(30);
        backMotor.setCANMaxRetries(5);
        backMotor.getEncoder().setPosition(0);
    }

    @Override
    public SparkMax getLeftMotor() {
        return leftMotor;
    }

    @Override
    public SparkMax getRightMotor() {
        return rightMotor;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (elevator == null)
            elevator = Elevator.getInstance();
        inputs.leftPositionRotations = leftMotor.getEncoder().getPosition(); // .getPosition();
        inputs.leftVelocityRPM = leftMotor.getEncoder().getVelocity();
        inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
        inputs.leftCurrentAmps = leftMotor.getOutputCurrent();
        inputs.leftTemperature = leftMotor.getMotorTemperature();

        inputs.rightPositionRotations = -rightMotor.getEncoder().getPosition();
        inputs.rightVelocityRPM = rightMotor.getEncoder().getPosition();
        inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
        inputs.rightTemperature = leftMotor.getMotorTemperature();
        inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
        inputs.elevatorTargetState = elevator.getTargetState();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        rightConfig.closedLoop.pid(kP, kI, kD);

        leftConfig.closedLoop.pid(kP, kI, kD);

    }

    @Override
    public void setPosition(double position) {
        if (position < 0) {
            position = 0;
        }
        leftMotor
                .getClosedLoopController()
                .setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward);
        rightMotor
                .getClosedLoopController()
                .setReference(-position, ControlType.kPosition, ClosedLoopSlot.kSlot0, -feedForward);
    }

    @Override
    public double getPosition() {
        return (leftMotor.getEncoder().getPosition() - rightMotor.getEncoder().getPosition()) / 2.0;
    }
}