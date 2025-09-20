package frc.robot.subsystems.Elevator;

public interface ElevatorIO{
    void setVoltage(double volts);
    void setPosition(double position);
    double getPosition();
    double getCurrent();
    double getVelocityMetersPerSec();
    void updateInputs(ElevatorIOInputs inputs);
    boolean atSetpoint();
    void configurePID(double kP, double kI, double kD);
    void updateSim();
}