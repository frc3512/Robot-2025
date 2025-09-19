package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leader;
    private final TalonFX follower;
    private static final double TICKS_PER_REV = 2048.0; // TalonFX integrated sensor

    public ElevatorIOTalonFX(int leaderID, int followerID) {
        leader = new TalonFX(leaderID);
        follower = new TalonFX(followerID);

        // Set follower to follow leader
        follower.setControl(new com.ctre.phoenix6.controls.Follower(leaderID, true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        double rotations = getPosition();
        double positionRad = rotations * 2.0 * Math.PI;
        inputs.data = new ElevatorIOData(positionRad);
    }

    public double getPosition() {
        return (leader.getRotorPosition().getValueAsDouble() + follower.getRotorPosition().getValueAsDouble()) / 2;
    }

    @Override
    public void runOpenLoop(double output) {
        leader.set(output); 
    }
}