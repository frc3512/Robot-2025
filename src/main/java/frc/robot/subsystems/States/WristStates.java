package frc.robot.subsystems.States;

public enum WristStates {
    
    // ! WRIST WILL ONLY HAVE TWO POSITIONS
    // * They have different names because it makes more sense in a naming sceme

    // | HORIZONTAL
    INTAKE(0.0),
    ALGAE(0.0),
    PROCESS(0.0),

    // | VERTICAL
    CORAL(0.0);

    public double position;

    WristStates(double totalPosition) {
        this.position = totalPosition;
    }

}
