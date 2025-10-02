package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class Automation extends SubsystemBase{
    
    private final Arm arm;
    private final Elevator elevator;
    private final Wrist wrist;

    private final Intake intake;

    boolean coralReady = false;
    boolean algaeReady = false;

    char piece = '!';

    public Automation() {
        arm = new Arm();
        elevator = new Elevator();
        wrist = new Wrist();

        intake = new Intake();
    }

    // * Reset / Defualt
    public Command reset() {
        return Commands.sequence(
            Commands.runOnce(() -> elevator.setClampedGoal(Constants.ElevatorConstants.stow)),
            Commands.runOnce(() -> arm.setClampedGoal(Constants.ArmConstants.stow)),
            Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.vertical)),
            Commands.runOnce(() -> coralReady = false),
            Commands.runOnce(() -> algaeReady = false) 
        );
    }

    // * Full Auto
    public Command autoScore(double pos) {
        return Commands.sequence(
            prepScore(pos),
            Commands.waitUntil(() -> coralReady == true),
            score()
        );
    }

    // * Coral 

    // | L2 ^
    public Command score() {
        return Commands.sequence(
            Commands.parallel(setPlace(), placeCoral()),
            Commands.waitUntil(() -> getPiece() == '!'), 
            Commands.runOnce(() -> reset())
        );
    }

    // | L1

    // | Intake

    // * Algae

    // | De-Reef

    // | Process

    // | Barge

    // | Intake
    public Command intakeCoral() {
        if (getPiece() == '!') {
            return Commands.sequence(
                Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.horizontal)),
                Commands.runOnce(() -> elevator.setClampedGoal(Constants.ElevatorConstants.intake)),
                Commands.runOnce(() -> arm.setClampedGoal(Constants.ArmConstants.intake)),
                Commands.waitUntil(() -> arm.atGoal()),
                grabCoral()
            );
        } else {
            return reset();
        }
    }

    public Command grabCoral() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.intakeCoral()),
            Commands.waitUntil(() -> getPiece() == 'C'),
            Commands.runOnce(() -> intake.stop())
        );
    }

    // * Prep

    // | Coral
    public Command prepScore(double pos) {
        if (getPiece() == 'C') {
            return Commands.sequence(
                Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.vertical)),
                Commands.waitUntil(() -> wrist.atGoal()),
                Commands.runOnce(() -> elevator.setClampedGoal(pos)),
                Commands.runOnce(() -> arm.setClampedGoal(Constants.ArmConstants.prepScore)),
                Commands.waitUntil(() -> elevator.atGoal()),
                Commands.waitUntil(() -> arm.atGoal()),
                Commands.runOnce(() -> coralReady = true)
            );
        } else {
            return reset();
        }
    }

    public Command prepCoral() {
        return Commands.sequence(
            Commands.runOnce(() -> elevator.setClampedGoal(Constants.ElevatorConstants.prepCoral)),
            Commands.runOnce(() -> arm.setClampedGoal(Constants.ArmConstants.stow)),
            Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.vertical))
        );
    }

    // | Algae

    // * Logic
    // Todo: Tune actual RGB Values

    private char getPiece() {
        if (hasCoral()) piece = 'C';
        if (hasAlgae()) piece = 'A';
        if (!hasCoral() && !hasAlgae()) piece = '!';

        return piece;
    }

    private boolean hasCoral() {
        boolean coralIn; 

        if (intake.getObjectDistance() >= 0.1 && intake.getObjectDistance() <= 0.3) {
            if (intake.getR() == 0.1 &&
                intake.getG() == 0.1 && 
                intake.getB() == 0.1) {
                
                coralIn = true;
            } else {
                coralIn = false;
            }
        } else {
            coralIn = false;            
        }

        return coralIn;
    }

    private boolean hasAlgae() {
        boolean algaeIn;

        if (intake.getObjectDistance() >= 0.1 && intake.getObjectDistance() <= 0.3) {
            if (intake.getR() == 0.1 &&
                intake.getG() == 0.1 && 
                intake.getB() == 0.1) {
                
                algaeIn = true;
            } else {
                algaeIn = false;
            }
        } else {
            algaeIn = false;            
        }

        return algaeIn;
    }

    // * Parallel

    private Command setPlace() {
        return Commands.runOnce(() -> arm.setClampedGoal(Constants.ArmConstants.score));
    }

    private Command placeCoral() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.placeCoral()),
            Commands.waitUntil(() -> getPiece() == '!'),
            Commands.runOnce(() -> intake.stop())
        );
    }

    // ----------

    @Override
    public void periodic() {
        // Log Logic
        DogLog.log("Has Coral", hasCoral());
        DogLog.log("Has Algae", hasAlgae());

        DogLog.log("Coral Is Ready", coralReady);
        DogLog.log("Algae Is Ready", algaeReady);

        DogLog.log("Current Peice", getPiece());

        // Update Vars
        getPiece();
    }
}
