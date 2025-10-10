package frc.robot.subsystems.States;

public enum ArmStates{

    // negative = forward
    
    // * Defualt
    STOW(0.0),

    // * Coral 

    // | Score
    PREP_CORAL(-28),
    PLACE_CORAL(-48),

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
    BARGE(0.47),

    // * Test
    FRONT(-40),
    BACK(40),
    MIDDLE(0);


    public double position;

    ArmStates(double totalPosition) {
        this.position = totalPosition;
    }

}
