package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputs inputs = new ElevatorIOInputs();
    private ElevatorStates state = ElevatorStates.STOW;

    private ElevatorStates targetLevel = ElevatorStates.STOW;

    public static Elevator instance;

    public static Elevator setInstance(int leadID, int followerID) {
        instance = new Elevator(leadID, followerID);
        return instance;
    }

    public Elevator(int leadID, int followerID) {
        this.io = new ElevatorIOTalonFX(leadID, followerID);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("Elevator", inputs);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }
    
    public static Elevator getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Elevator instance not set");
        }
        return instance;
    }

    public void setTargetState(ElevatorStates newState) {
        targetLevel = newState;
    }

    public void setState() {
        io.setPosition(targetLevel.position);
        state = targetLevel;
    }

    public ElevatorStates getTargetState() {
        return targetLevel;
    }

    public double getPosition() {
        return inputs.position;
    }

    public double getVelocity() {
        return inputs.velocityMetersPerSec;
    }

    public double getCurrent() {
        return inputs.currentAmps;
    }

    public ElevatorStates getState() {
        return state;
    }

    public boolean atSetpoint() {
        return inputs.isAtSetpoint;
    }

    public void setState(ElevatorStates newState) {
        state = newState;
    }
}