package frc.robot.subsystems.States;

public enum ElevatorStates {

    // * Defualt
    STOW(0.0),
 
    // * Coral

    // | Reef
    L4(0.0),
    L3(0.0),
    L2(0.0),
    L1(0.0),

    // | Intake
    INTAKE(0.0),
    PREP_CORAL(0.0),

    // * Algae

    // | De-Reef
    ALGAE_L1(0.0),
    ALGAE_L2(0.0),

    // | Prep
    ALGAE_STOW(0.0),

    // | Barge
    BARGE(0.0),

    // * Testing
    TEST_1(25.0);

    public final double position;

    ElevatorStates(double totalPosition) {
        this.position = totalPosition;
    }

}
