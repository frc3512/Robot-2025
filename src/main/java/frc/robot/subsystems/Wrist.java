package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.States.WristStates;

public class Wrist extends SubsystemBase{

    private final TalonFX motor;

    private double statorLimit = 20;
    private double supplyLimit = 20;

    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(statorLimit)
    .withSupplyCurrentLimit(supplyLimit)
    .withStatorCurrentLimitEnable(true)
    .withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentLowerLimit(supplyLimit)
    .withSupplyCurrentLowerTime(0);

    private PositionVoltage positionRequest = new PositionVoltage(WristStates.CORAL.position);

    private double desiredState;

    public Wrist() {

        motor = new TalonFX(Constants.WristConstants.motorID);

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = Constants.WristConstants.GEAR_RATIO;
        config.Slot0.withKP(Constants.WristConstants.kP);

        motor.setPosition(0.00000);

        motor.getConfigurator().apply(config);
    }

    public void setClampedGoal(WristStates goal) {
        desiredState = MathUtil.clamp(goal.position, -95, 95);
    }

    public boolean atSetpoint() {
        double positionError = 
            Math.abs((motor.getClosedLoopError().getValueAsDouble()));
        return positionError * 360.0 < 2; // 1 degree of error, make smaller if needed
      }

    @Override
    public void periodic() {
        // Log PID
        DogLog.log("Wrist/Degrees", motor.getPosition().getValueAsDouble() * 360);

        DogLog.log("Wrist/Wrist Goal", desiredState);
        DogLog.log("Wrist/Wrist Voltage", motor.getStatorCurrent().getValueAsDouble());

        DogLog.log("Wrist/At Goal", atSetpoint());

        // Log Basics
        DogLog.log("Wrist/Temp", motor.getDeviceTemp().getValueAsDouble());

        motor.setControl(positionRequest.withPosition(desiredState / 360.0));
    }
}