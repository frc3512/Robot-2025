package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.States.ArmStates;

public class Arm extends SubsystemBase{

    private final TalonFX motor;

    private final Canandmag encoder;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private PositionVoltage positionRequest = new PositionVoltage(ArmStates.STOW.position);

    private double desiredState;

    private static double clamp(double height) {
        return MathUtil.clamp(height, 0.001, 0.7);
    }

    public Arm() {
        encoder = new Canandmag(30);

        motor = new TalonFX(15);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.SensorToMechanismRatio = Constants.ArmConstants.GEAR_RATIO;

        config.Slot0.withKP(Constants.ArmConstants.kP);
        config.Slot0.withKG(Constants.ArmConstants.kG);

        config.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);

    }

    // Tune Clamp High/Low
    public void setClampedGoal(ArmStates goal) {
        desiredState = clamp(goal.position);
    } 

    public double getPosition() {
        return encoder.getAbsPosition();
    }

    public boolean atGoal() {
        double posError = 
          Math.abs(motor.getClosedLoopError().getValueAsDouble());
    
        return posError < Constants.ArmConstants.tolerance;
    }

    @Override
    public void periodic() {
        // Log PID
        DogLog.log("Arm/Arm Pos", getPosition());
        DogLog.log("Arm/Arm Goal", desiredState);
        DogLog.log("Arm/Arm Voltage", motor.getStatorCurrent().getValueAsDouble());

        // Log Basics
        DogLog.log("Arm/Temp", motor.getDeviceTemp().getValueAsDouble());
    }
}