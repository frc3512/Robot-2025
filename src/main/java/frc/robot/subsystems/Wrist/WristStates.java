package frc.robot.subsystems.Wrist;

public enum WristStates {
    
    // * Positions for the wrist using encoder ticks *
    // * TUNE ALL POSITIONS, THESE ARE JUST ROUGH ESTIMATES *

    GRAB(0.0),
    HOLD(1);

    public double position;

    private WristStates(double position) {
        this.position = position;
    }

}
