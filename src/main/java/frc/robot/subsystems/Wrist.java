package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase{

    private final TalonFX motor;

    private final TalonFXConfiguration config;

    public Wrist() {

        motor = new TalonFX(15);

        config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = 12.5 / 1.0;

        motor.getConfigurator().apply(config);

        motor.setPosition(0.000);

    }

    // Tune Clamp High/Low
    // public void setClampedGoal(double pos) {
    //     setGoal(MathUtil.clamp(pos, 0.0, 0.7));
    // }

    public boolean atGoal() {
        double posError = 
            motor.getClosedLoopError().getValueAsDouble();
        
        return posError < Constants.WristConstants.tolerance;
    }

    public double getPosition() {
        StatusSignal<Angle> rawPos = motor.getPosition();
        double pos = rawPos.getValueAsDouble();
        return pos;
    }

    @Override
    public void periodic() {
        super.periodic();

        // Log PID
        DogLog.log("Wrist/Wrist Pos", getPosition());
        // DogLog.log("Wrist/Wrist Goal", );
        DogLog.log("Wrist/Wrist Voltage", motor.getStatorCurrent().getValueAsDouble());

        // Log Basics
        DogLog.log("Wrist/Motor Temp", motor.getDeviceTemp().getValueAsDouble());
    }

}