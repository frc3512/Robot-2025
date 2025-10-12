package frc.robot.subsystems.States;

public enum ArmStates{

    // negative = forward
    
    // * Defualt
    STOW(0.0),

    // * Coral 

    // | Score
    PREP_CORAL(-28),
    PLACE_CORAL(-48),

    TROUGH(-75),

    // | Intake
    INTAKE(117),
    INTAKE_ALGAE(75),

    // | Prep
    HOLD_CORAL(-10),

    // * Algae

    // | De-Reef
    REMOVE_ALGAE(-65),

    // | Prep 
    PREP_ALGAE(0.0),

    // | Process
    PROCESS(-80),

    // | Barge
    BARGE(20),

    // * Test
    FRONT(-40),
    BACK(40),
    MIDDLE(0);


    public double position;

    ArmStates(double totalPosition) {
        this.position = totalPosition;
    }

}
