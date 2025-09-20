package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputs implements LoggableInputs{

    public double position = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temp = 0.0;
    public boolean isAtSetpoint = false;

    @Override
    public void toLog(LogTable table) {
        table.put("Elevator/Position", position);
        table.put("Elevator/VelocityMetersPerSec", velocityMetersPerSec);
        table.put("Elevator/AppliedVolts", appliedVolts);
        table.put("Elevator/CurrentAmps", currentAmps);
        table.put("Elevator/Temperature", temp);
        table.put("Elevator/AtSetpoint", isAtSetpoint);
    }
    @Override
    public void fromLog(LogTable table) {}
}