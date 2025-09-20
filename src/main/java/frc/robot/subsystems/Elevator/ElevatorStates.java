package frc.robot.subsystems.Elevator;

public enum ElevatorStates {

    // * Positions for the elevator using encoder ticks *
    
    // | Defualt Positions |
    STOW(1),
    HP(5),
    
    // | Coral Scoring Positions |
    L1(1),
    L2(3),
    L3(6),
    L4(9),

    // | Algae Positions |
    ALGAE_L1(2),
    ALGAE_L2(4),
    ALGAE_STOW(2),
    BARGE(9);

    public double position;

    ElevatorStates(double totalPosition) {
        this.position = totalPosition;
    }
}