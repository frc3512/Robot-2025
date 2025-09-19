package frc.robot.subsystems.Elevator;

public class ElevatorIOSim implements ElevatorIO {
    private double positionRad = 0.0;
    private double velocityRadPerSec = 0.0;
    private double lastOutput = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Simple simulation: output directly affects velocity, which updates position
        velocityRadPerSec = lastOutput * 0.05;
        positionRad += velocityRadPerSec * 0.02; // 20ms loop
        if (positionRad < 0) positionRad = 0;
        inputs.data = new ElevatorIOData(positionRad);
    }

    @Override
    public void runOpenLoop(double output) {
        lastOutput = output;
    }
}