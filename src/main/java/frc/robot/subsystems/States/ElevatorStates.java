package frc.robot.subsystems.States;

public enum ElevatorStates {

    // * Defualt
    STOW(0.0),
 
    // * Coral

    // | Reef
    L4(44.44),
    L3(20.01),
    L2(3.89),
    L1(7.84),

    // | Intake
    INTAKE(9.39),
    PREP_CORAL(0.0),

    // * Algae

    // | De-Reef
    ALGAE_L1(32.93),
    ALGAE_L2(46.25),

    // | Prep
    ALGAE_STOW(9.22),

    // | Barge
    BARGE(54.08),

    // * Testing
    TEST_1(25.0);

    public final double position;

    ElevatorStates(double totalPosition) {
        this.position = totalPosition;
    }

}
