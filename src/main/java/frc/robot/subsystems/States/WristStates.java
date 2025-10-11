package frc.robot.subsystems.States;

public enum WristStates {
    
    // ! WRIST WILL ONLY HAVE TWO POSITIONS
    // * They have different names because it makes more sense in a naming sceme
    // * Positions in degrees

    // | HORIZONTAL
    INTAKE(-2),
    ALGAE(-2),
    PROCESS(-2),

    // | VERTICAL
    CORAL(92);

    public double position;

    WristStates(double totalPosition) {
        this.position = totalPosition;
    }

}
