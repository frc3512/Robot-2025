package frc.robot.subsystems.Elevator;

public interface ElevatorIO {
    class ElevatorIOInputs {
        public ElevatorIOData data = new ElevatorIOData(0.0);
    }

    record ElevatorIOData(double positionRad) {}

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void runOpenLoop(double output) {}
}