package frc.robot.subsystems.States;

public enum WristStates {
    
    // ! WRIST WILL ONLY HAVE TWO POSITIONS
    // * They have different names because it makes more sense in a naming sceme
    // * Positions in degrees

    // | HORIZONTAL
    INTAKE(95),
    ALGAE(95),
    PROCESS5(95),
    TROUGH(95),

    // | VERTICAL
    CORAL(0);

    public double position;

    WristStates(double totalPosition) {
        this.position = totalPosition;
    }

}
