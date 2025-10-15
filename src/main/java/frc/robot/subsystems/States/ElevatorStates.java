package frc.robot.subsystems.States;

public enum ElevatorStates {

    // * Values are in inches, conversion happens in Elevator.java

    // * Defualt
    STOW(0.0),
 
    // * Coral

    // | Reef
    L4(46),
    L3(18),
    L2(0),
    L1(7),

    // | Intake
    INTAKE(11),
    PREP_CORAL(0),

    // * Algae

    // | De-Reef
    ALGAE_L1(15),
    ALGAE_L2(28),

    // | Intake
    ALGAE_INTAKE(0),

    // | Prep
    ALGAE_STOW(9),

    // | Barge
    BARGE(60),

    // * Testing
    TEST_1(25);

    public final double position;

    ElevatorStates(double totalPosition) {
        this.position = totalPosition;
    }

}
