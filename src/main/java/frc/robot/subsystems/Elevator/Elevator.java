package frc.robot.subsystems.Elevator;

public class Elevator {
    private final ElevatorIO io;
    private int goalPositionTicks = 0;

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    // Set goal in encoder ticks
    public void setGoalTicks(int positionTicks) {
        goalPositionTicks = positionTicks;
    }

    public void periodic() {
        ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();
        io.updateInputs(inputs);

        int error = goalPositionTicks - inputs.data.positionRad();
        double kP = 0.01; // Gain depends on your mechanism
        double output = kP * error;
        io.runOpenLoop(output);
    }
}