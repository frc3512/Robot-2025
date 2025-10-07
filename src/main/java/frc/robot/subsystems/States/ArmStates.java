package frc.robot.subsystems.States;

public enum ArmStates{
    
    // * Defualt
    STOW(0.0),

    // * Coral 

    // | Score
    PREP_CORAL(0.472),
    PLACE_CORAL(0.509),

    TROUGH(0.59),

    // | Intake
    INTAKE(0.003),
    HP(0.0),

    // * Algae

    // | De-Reef
    REMOVE_ALGAE(0.65),

    // | Prep 
    PREP_ALGAE(0.0),

    // | Process
    PROCESS(0.0),

    // | Barge
    BARGE(0.47);

    public double position;

    ArmStates(double totalPosition) {
        this.position = totalPosition;
    }

}
