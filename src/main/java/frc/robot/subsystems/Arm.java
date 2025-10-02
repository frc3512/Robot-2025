package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.lib.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Arm extends ProfiledPIDSubsystem{

    private final TalonFX motor;

    private final Canandmag encoder;

    public Arm() {
        super(
            new ProfiledPIDController(
            Constants.ArmConstants.kP,
            Constants.ArmConstants.kI,
            Constants.ArmConstants.kD,
            Constants.ArmConstants.constraints));
        getController().setTolerance(Constants.ArmConstants.tolerance);

        encoder = new Canandmag(30);

        motor = new TalonFX(15);
        motor.setNeutralMode(NeutralModeValue.Brake);

        enable();
    }

    // Tune Clamp High/Low
    public void setClampedGoal(double pos) {
        setGoal(MathUtil.clamp(pos, 0.005, 0.7));
    }

    public double getPosition() {
        return encoder.getAbsPosition();
    }

    public boolean atGoal() {
        double posError = 
          Math.abs(getController().getPositionError());
    
        return posError < Constants.ArmConstants.tolerance;
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
       return encoder.getAbsPosition();
    }
}