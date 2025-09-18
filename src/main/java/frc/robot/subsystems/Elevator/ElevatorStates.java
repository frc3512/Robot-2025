package frc.robot.subsystems.Elevator;

public enum ElevatorStates {

    // * Positions for the elevator using encoder ticks *
    
    // | Defualt Positions |
    STOW(1),
    HP(5),
    
    // | Coral Scoring Positions |
    L1(8.7),
    L2(14.2),
    L3(26),
    L4(46),

    // | Algae Positions |
    ALGAE_L1(3.5),
    ALGAE_L2(15.84),
    ALGAE_STOW(14.5);

    public double position;

    ElevatorStates(double totalPosition) {
        this.position = totalPosition;
    }

}
