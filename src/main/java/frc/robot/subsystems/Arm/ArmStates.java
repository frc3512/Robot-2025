package frc.robot.subsystems.Arm;

public enum ArmStates {

    // TODO: TUNE ALL POSITIONS, THESE ARE JUST ROUGH ESTIMATES

    // * Positions for the arm using encoder ticks *
 
    // | Defualt Positions |
    STOW(0.4),

    // | Coral Scoring Positions |
    FRONT_SCORE(0.6),

    // | Intake Positions |
    REAR_INTAKE(0.1),
    FRONT_INTAKE(0.7),

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
