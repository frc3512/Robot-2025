package frc.robot.subsystems.Arm;

public enum ArmStates {

    // TODO: TUNE ALL POSITIONS, THESE ARE JUST ROUGH ESTIMATES

    // * Positions for the arm using encoder ticks *
 
    // | Defualt Positions |
    STOW(0.4),

    // | Coral Scoring Positions |
    FRONTSCORE(0.6),
    BACKSCORE(0.37),

    // | Intake Positions |
    REARINTAKE(0.1),
    FRONTINTAKE(0.7),

    // | Algae Positions |
    ALGAE_STOW(0.42),
    ALGAE_REMOVE(0.67),
    ALGAE_BARGE(0.55),
    ALGAE_PROCESS(0.68);
    
    public double position;

    ArmStates(double totalPosition) {
      this.position = totalPosition;
    }
}
