package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.States.ArmStates;
import frc.robot.subsystems.States.ElevatorStates;

public class Automation extends SubsystemBase{
    
    private final Arm arm;
    private final Elevator elevator;
    private final Wrist wrist;

    private final Intake intake;

    boolean coralReady = false;
    boolean bargeReady = false;
    boolean processorReady = false;

    char piece = 'C';

    public Automation() {
        arm = new Arm();
        elevator = new Elevator();
        wrist = new Wrist();

        intake = new Intake();
    }

    // * Reset / Defualt
    // public Command reset() {
    //     return Commands.sequence(
    //         Commands.runOnce(() -> intake.stop()), 
    //         Commands.runOnce(() -> elevator.setClampedGoal(Constants.ElevatorConstants.stow)),
    //         Commands.runOnce(() -> arm.setClampedGoal(Constants.ArmConstants.stow)),
    //         Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.vertical)),
    //         Commands.runOnce(() -> coralReady = false),
    //         Commands.runOnce(() -> bargeReady = false),
    //         Commands.runOnce(() -> processorReady = false)
    //     );
    // }

    // * Full Auto

    // | Coral
    // public Command autoScore(double pos) {
    //     return Commands.sequence(
    //         prepScore(pos),
    //         Commands.waitUntil(() -> coralReady == true),
    //         score()
    //     );
    // }

    // * Coral 

    // | L2 ^
    // public Command score() {
    //     return Commands.sequence(
    //         Commands.parallel(setPlace(), placeCoral()),
    //         Commands.waitUntil(() -> getPiece() == '!'), 
    //         Commands.runOnce(() -> reset())
    //     );
    // }

    // | L1
    // public Command trough() {
    //     return Commands.sequence(
    //         Commands.runOnce(() -> elevator.setClampedGoal(Constants.ElevatorConstants.l1)),
    //         Commands.waitUntil(() -> elevator.atGoal()),
    //         Commands.runOnce(() -> arm.setClampedGoal(Constants.ArmConstants.trough)),
    //         Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.horizontal)),
    //         Commands.waitUntil(() -> arm.atGoal()),
    //         Commands.waitUntil(() -> wrist.atGoal()),
    //         Commands.runOnce(() -> intake.placeCoral()),
    //         Commands.waitSeconds(0.5),
    //         Commands.runOnce(() -> intake.stop()),
    //         reset()
    //        // * Command is Automatic!!
    //     );
    // }   

    // | Intake
    // public Command intakeCoral() {
    //     if (getPiece() == '!') {
    //         return Commands.sequence(
    //             Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.horizontal)),
    //             Commands.runOnce(() -> elevator.setClampedGoal(Constants.ElevatorConstants.intake)),
    //             Commands.runOnce(() -> arm.setClampedGoal(Constants.ArmConstants.intake)),
    //             Commands.waitUntil(() -> arm.atGoal()),
    //             Commands.waitUntil(() -> wrist.atGoal()),
    //             grabCoral()
    //         );
    //     } else {
    //         return reset();
    //     }
    // }

    public Command grabCoral() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.intakeCoral()),
            Commands.waitUntil(() -> getPiece() == 'C'),
            Commands.runOnce(() -> intake.stop())
        );
    }

    // * Algae

    // | De-Reef
    // public Command grabAlgae(double level) {
    //     if (getPiece() == '!') {
    //         return Commands.sequence(
    //             Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.horizontal)),
    //             Commands.runOnce(() -> elevator.setClampedGoal(level)),
    //             Commands.runOnce(() -> arm.setClampedGoal(Constants.ArmConstants.algae)),
    //             grabAlgae(),
    //             prepAlgae()
    //         );
    //     } else {
    //         return reset();
    //     }
    // }

    // | Process

    // | Barge
    // public Command scoreBarge() {
    //     if (getPiece() == 'A') {
    //         return Commands.sequence(
    //             Commands.runOnce(() -> intake.outtake()),
    //             Commands.waitUntil(() -> getPiece() == '!'),
    //             reset()
    //         );
    //     } else {
    //         return reset();
    //     }
    // }

    // | Intake
    // public Command intakeAlgae() {
    //     if (getPiece() == '!') {
    //         return Commands.sequence(
    //             Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.horizontal)),
    //             Commands.runOnce(() -> elevator.setClampedGoal(Constants.ElevatorConstants.intake)),
    //             Commands.runOnce(() -> arm.setClampedGoal(Constants.ArmConstants.intake)),
    //             Commands.waitUntil(() -> arm.atGoal()),
    //             Commands.waitUntil(() -> wrist.atGoal()),
    //             grabAlgae()
    //         );
    //     } else {
    //         return reset();
    //     }
    // }

    public Command grabAlgae() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.intakeAlgae()),
            Commands.waitUntil(() -> getPiece() == 'A'),
            Commands.runOnce(() -> intake.hold())
        );
    }

    // * Prep

    // | Coral
    // public Command prepScore(double pos) {
    //     if (getPiece() == 'C') {
    //         return Commands.sequence(
    //             Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.vertical)),
    //             Commands.waitUntil(() -> wrist.atGoal()),
    //             Commands.runOnce(() -> elevator.setClampedGoal(pos)),
    //             Commands.runOnce(() -> arm.setClampedGoal(Constants.ArmConstants.prepScore)),
    //             Commands.waitUntil(() -> elevator.atGoal()),
    //             Commands.waitUntil(() -> arm.atGoal()),
    //             Commands.runOnce(() -> coralReady = true)
    //         );
    //     } else {
    //         return reset();
    //     }
    // }

    // public Command prepCoral() {
    //     return Commands.sequence(
    //         Commands.runOnce(() -> elevator.setClampedGoal(Constants.ElevatorConstants.prepCoral)),
    //         Commands.runOnce(() -> arm.setClampedGoal(Constants.ArmConstants.stow)),
    //         Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.vertical))
    //     );
    // }

    // | Algae
    // public Command prepAlgae() {
    //     if (getPiece() == 'A') {
    //         return Commands.sequence(
    //             Commands.runOnce(() -> elevator.setClampedGoal(Constants.ElevatorConstants.aStow)),
    //             Commands.runOnce(() -> arm.setClampedGoal(Constants.ArmConstants.stow)),
    //             Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.horizontal))
    //         );
    //     } else {
    //         return reset();
    //     }
    // }

    // public Command prepBarge() {
    //     if (getPiece() == 'A') {
    //         return Commands.sequence(
    //             Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.horizontal)),
    //             Commands.runOnce(() -> elevator.setClampedGoal(Constants.ElevatorConstants.barge)),
    //             Commands.waitUntil(() -> elevator.atGoal()),
    //             Commands.runOnce(() -> arm.setClampedGoal(Constants.ArmConstants.barge)),
    //             Commands.runOnce(() -> bargeReady = true)
    //         );
    //     } else {
    //         return reset();
    //     }
    // }

    // * Logic
    // Todo: Tune actual RGB Values

    private char getPiece() {
        if (hasCoral()) piece = 'C';
        if (hasAlgae()) piece = 'A';
        if (!hasCoral() && !hasAlgae()) piece = '!';

        return piece;
    }

    private boolean hasCoral() {
        if (intake.getObjectDistance() <= 0.04) {
            if (intake.getR() >= 0.29 && intake.getR() <= 0.36 &&
                intake.getG() >= 0.40 && intake.getG() <= 0.49 && 
                intake.getB() >= 0.14 && intake.getB() <= 0.20) {
                
                return true;
            } else {
                return false;
            }
        } else {
            return false;            
        }
    }

    private boolean hasAlgae() {
        if (intake.getObjectDistance() <= 0.07) {
            if (intake.getR() >= 0.04 && intake.getR() <= 0.12 && 
                intake.getG() >= 0.30 && intake.getG() <= 0.36 && 
                intake.getB() >= 0.10 && intake.getB() <= 0.18) {
                
                return true;
            } else {
                return false;
            }
        } else {
            return false;            
        }
    }

    // * Parallel

    private Command setPlace() {
        return Commands.runOnce(() -> arm.setClampedGoal(ArmStates.PLACE_CORAL));
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
        DogLog.log("Barge Is Ready", bargeReady);
        DogLog.log("Process Is Ready", processorReady);
        
        DogLog.log("Current Peice", getPiece());
        
        // Update Piece
        getPiece();
        hasAlgae();
        hasCoral();
    } 
    
    // ! TESTING ONLY COMMANDS

    public void test1() {
        elevator.setClampedGoal(ElevatorStates.TEST_1);
    }

    public void down() {
        elevator.setClampedGoal(ElevatorStates.STOW);
    }

    public void l4() {
        elevator.setClampedGoal(ElevatorStates.L4);
    }

    public void l3() {
        elevator.setClampedGoal(ElevatorStates.L3);
    }
    
    public void l2() {
        elevator.setClampedGoal(ElevatorStates.L2);
    }

    public void intake() {
        intake.intakeCoral();
    }

    public void outtake() {
        intake.outtake();
    }

    public void stopRollers() {
        intake.stop();
    }
}