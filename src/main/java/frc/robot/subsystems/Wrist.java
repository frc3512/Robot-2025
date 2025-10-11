package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.States.ArmStates;
import frc.robot.subsystems.States.WristStates;

public class Wrist extends SubsystemBase{

    private final TalonFX motor;

    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private PositionVoltage positionRequest = new PositionVoltage(ArmStates.STOW.position);

    private double desiredState;

    public Wrist() {

        motor = new TalonFX(15);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Feedback.SensorToMechanismRatio = Constants.ArmConstants.GEAR_RATIO;

        config.Slot0.withKP(Constants.ArmConstants.kP);
        config.Slot0.withKD(Constants.ArmConstants.kD);
        config.Slot0.withKA(Constants.ArmConstants.kA);

        motor.setPosition(0.00000);

        motor.getConfigurator().apply(config);
    }

    public void setClampedGoal(WristStates goal) {
        desiredState = MathUtil.clamp(goal.position, 0, 90);
    }

    public boolean atSetpoint() {
        double positionError = 
            Math.abs((motor.getClosedLoopError().getValueAsDouble()));
        return positionError * 360.0 < 1; // 5 degrees of error, make smaller if needed
      }

    @Override
    public void periodic() {
        // Log PID
        DogLog.log("Arm/Arm Real Deg", motor.getPosition().getValueAsDouble() * 360);

        DogLog.log("Arm/Arm Goal", desiredState);
        DogLog.log("Arm/Arm Voltage", motor.getStatorCurrent().getValueAsDouble());

        DogLog.log("Arm/At Goal", atSetpoint());

        // Log Basics
        DogLog.log("Arm/Temp", motor.getDeviceTemp().getValueAsDouble());

        motor.setControl(positionRequest.withPosition(desiredState / 360.0));
    }
}