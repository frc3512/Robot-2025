package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import frc.lib.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Wrist extends ProfiledPIDSubsystem{

    private final TalonFX motor;

    public Wrist() {
        super(
            new ProfiledPIDController(
            Constants.ArmConstants.kP,
            Constants.ArmConstants.kI,
            Constants.ArmConstants.kD,
            Constants.ArmConstants.constraints));
        getController().setTolerance(Constants.ArmConstants.tolerance);

        motor = new TalonFX(15);
        motor.setNeutralMode(NeutralModeValue.Brake);

        enable();
    }

    // Tune Clamp High/Low
    public void setClampedGoal(double pos) {
        setGoal(MathUtil.clamp(pos, 0.0, 0.7));
    }

    public boolean atGoal() {
        double posError = 
            getController().getPositionError();
        
        return posError < Constants.WristConstants.tolerance;
    }

    public double getPosition() {
        StatusSignal<Angle> rawPos = motor.getPosition();
        double pos = rawPos.getValueAsDouble();
        return pos;
    }

    @Override
    public void periodic() {
        // Log PID
        DogLog.log("Arm Pos", getPosition());
        DogLog.log("Arm Goal", getController().getGoal().position);
        DogLog.log("Arm Voltage", motor.getStatorCurrent().getValueAsDouble());

        // Log Basics
        DogLog.log("Temp", motor.getDeviceTemp().getValueAsDouble());
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        motor.setVoltage(output);
    }

    @Override
    protected double getMeasurement() {
        StatusSignal<Angle> rawPos = motor.getPosition();
        double pos = rawPos.getValueAsDouble();
        return pos;
    }
}