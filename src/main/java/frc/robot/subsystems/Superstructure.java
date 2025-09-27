package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Superstructure {

    // | | Subsystem Objects
    public final Elevator elevator = new Elevator();
    public final Groundtake groundtake = new Groundtake();
    public final Reeftake reeftake = new Reeftake();
    public final Climber climber = new Climber();
    public final LED leds = new LED();

    public Superstructure() {}

    public void l4() {
        elevator.setClampedGoal(Constants.ElevatorConstants.l4Pos);
    }

    public void l3() {
        elevator.setClampedGoal(Constants.ElevatorConstants.l3Pos);
    }

    public void l2() {
        elevator.setClampedGoal(Constants.ElevatorConstants.l2Pos);
    }

    public void l1() {
        elevator.setClampedGoal(Constants.ElevatorConstants.l1Pos);
    }

    public void stow() {
        elevator.setClampedGoal(Constants.ElevatorConstants.stowPos);
    }

    public void HP() {
        elevator.setClampedGoal(Constants.ElevatorConstants.hpPos);
    }


    public void aStow() {
        elevator.setClampedGoal(Constants.ElevatorConstants.aStowPos);
    }

    public void LEDLogic() {
        if (climber.isBeamBroken()) {
            leds.setPattern(leds.purple);
          } else if (!reeftake.isCoralIn()) {
            leds.setPattern(leds.scrollngRainbow);
          } else {
            leds.setPattern(leds.red);
          }
    }

    public SequentialCommandGroup a1() {
    return new InstantCommand(() -> elevator.a1())
        .andThen(new InstantCommand(() -> reeftake.algaeIntake()));
  }

  public SequentialCommandGroup a2() {
    return new InstantCommand(() -> elevator.a2())
        .andThen(new InstantCommand(() -> reeftake.algaeIntake()));
  }

  public SequentialCommandGroup scoreBarge() {
    return new InstantCommand(() -> reeftake.algaeOuttake())
        .andThen(new WaitCommand(0.375))
        .andThen(new InstantCommand(() -> elevator.stow()))
        .andThen(new InstantCommand(() -> reeftake.algaeStop()));
  }

  public SequentialCommandGroup retractAlgae() {
    return new InstantCommand(() -> elevator.aStow())
        .andThen(new InstantCommand(() -> reeftake.algaeStop()));
  }

  public SequentialCommandGroup score() {
    return new InstantCommand(() -> reeftake.coralIntake())
        .andThen(new WaitCommand(0.75))
        .andThen(new InstantCommand(() -> elevator.hp()))
        .andThen(new InstantCommand(() -> reeftake.coralStop()));
  }

  public SequentialCommandGroup intake() {
    return new InstantCommand(() -> elevator.hp()).andThen(reeftake.autoIntake());
  }

  public void climbUp() {
    climber.setClimber(0.8)
        .until(() -> climber.climberAtTop() == false)
        .andThen(climber.setClimber(0.0));
  }

  public void climbDown() {
    climber.setClimber(-0.8)
        .until(() -> climber.climberAtBottom() == false)
        .andThen(climber.setClimber(0.0));
  }  

  public void stopClimb() {
    climber.setClimber(0.0);
  }

  public void algaeOuttake() {
    reeftake.algaeOuttake();
  }

  public void algaeStop() {
    reeftake.algaeStop();
  }

  public void extendPivot() {
    groundtake.extendPivot();
  }

  public void retractPivot() {
    groundtake.retractPivot();
  }

  public void floorAlgaeIntake() {
    groundtake.floorAlgaeIntake();
  }

  public void floorAlgaeStop() {
    groundtake.floorAlgaeStop();
  }

  public void floorAlgaeOuttake() {
    groundtake.floorAlgaeOuttake();
  }

  public void keepAlgae() {
    groundtake.keepAlgae();
  }

  public void setMarkers() {}
}
