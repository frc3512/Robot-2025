package frc.robot.subsystems.States;

public enum ElevatorStates {

    // * Defualt
    STOW(0.2),
 
    // * Coral

    // | Reef
    L4(37.39),
    L3(15.56),
    L2(4.09),
    L1(9.05),

    // | Intake
    INTAKE(10.27),
    PREP_CORAL(0.0),

    // * Algae

    // | De-Reef
    ALGAE_L1(28.75),
    ALGAE_L2(39.12),

    // | Prep
    ALGAE_STOW(5.3),

    // | Barge
    BARGE(43.6);

    public final double position;

    ElevatorStates(double totalPosition) {
        this.position = totalPosition;
    }

}
