package frc.robot.subsystems.States;

public enum ArmStates{

    // negative = forward
    
    // * Defualt
    STOW(0.0),

    // * Coral 

    // | Score
    PREP_CORAL(-28),
    PLACE_CORAL(-48),

    TROUGH(-59),

    // | Intake
    INTAKE(100),
    HOLD_CORAL(-10),

    // * Algae

    // | De-Reef
    REMOVE_ALGAE(-65),

    // | Prep 
    PREP_ALGAE(0.0),

    // | Process
    PROCESS(-100),

    // | Barge
    BARGE(-47),

    // * Test
    FRONT(-40),
    BACK(20),
    MIDDLE(0);


    public double position;

    ArmStates(double totalPosition) {
        this.position = totalPosition;
    }

}
