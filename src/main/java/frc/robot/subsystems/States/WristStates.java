package frc.robot.subsystems.States;

public enum WristStates {
    
    // ! WRIST WILL ONLY HAVE TWO POSITIONS
    // * They have different names because it makes more sense in a naming sceme
    // * Positions in degrees
    // * use an overshoot to ensure contact with hardstops

    // | HORIZONTAL
    INTAKE(90),
    ALGAE(90),
    TROUGH(90),
    
    // | VERTICAL
    CORAL(0),
    
    // DIAGONAL
    PROCESS(45);

    public double position;

    WristStates(double totalPosition) {
        this.position = totalPosition;
    }

}
