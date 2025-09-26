package frc.robot.subsystems.Wrist;

public enum WristStates {
    
    // * Positions for the wrist using encoder ticks *
    // * TUNE ALL POSITIONS, THESE ARE JUST ROUGH ESTIMATES *

    VERTICAL(0.0),
    HORIZONTAL(1);

    public double position;

    private WristStates(double position) {
        this.position = position;
    }

}
