package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.DriveConstants;
import frc.robot.Auton.Autos;

public class Superstructure {

    private double maxSpeed = DriveConstants.maxSpeed;
    private double maxAngularRate = DriveConstants.maxAngularRate;
    private double slowSpeed = DriveConstants.slowSpeed;
    private double slowAngularRate = DriveConstants.slowAngularRate;

    private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * 0.1)
        .withRotationalDeadband(maxAngularRate * 0.07) // * Add a 7% deadband
        .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.FieldCentric driveSlow =
      new SwerveRequest.FieldCentric()
        .withDeadband(slowSpeed * 0.1)
        .withRotationalDeadband(slowAngularRate * 0.07) // * Add a 7% deadband
        .withDriveRequestType(DriveRequestType.Velocity);

    public final Vision visionElevator =
        new Vision(Constants.VisionConstants.elevatorCam, Constants.VisionConstants.elevatorCamOffset);
    public final Vision visionClimber =
        new Vision(Constants.VisionConstants.climberCam, Constants.VisionConstants.climberCamOffset);

    // | Controller Objects
    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandJoystick appendageJoystick = new CommandJoystick(1);

    public final Swerve drivetrain = DriveConstants.createDrivetrain();

    public final Superstructure actions = new Superstructure();
  
    // | | Subsystem Objects
    public final Elevator elevator = new Elevator();
    public final Groundtake groundtake = new Groundtake();
    public final Reeftake reeftake = new Reeftake();
    public final Climber climber = new Climber();
    public final LED leds = new LED();

    private final Autos autos;

    public Superstructure() {
      autos = new Autos(drivetrain);
      configureActions();
    }

    public void configureActions() {

      // * -- Controler Bindings --
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(
          () ->
              drive
                  .withVelocityX(-controller.getLeftY() * maxSpeed)
                  .withVelocityY(-controller.getLeftX() * maxSpeed)
                  .withRotationalRate(-controller.getRightX() * maxAngularRate)));

    controller
      .b()
      .whileTrue(
          drivetrain.applyRequest(
              () ->
                  driveSlow
                      .withVelocityX(-controller.getLeftY() * slowSpeed)
                      .withVelocityY(-controller.getLeftX() * slowSpeed)
                      .withRotationalRate(-controller.getRightX() * slowAngularRate)));

   // controller.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // | Aiming Controls
    controller.leftBumper().onTrue(drivetrain.selectReef("Left"));
    controller.rightBumper().onTrue(drivetrain.selectReef("Right"));

    controller.y().onTrue(drivetrain.selectPiece("Coral"));
    controller.a().onTrue(drivetrain.selectPiece("Algae"));

    controller.x().whileTrue(autoAim());

  // | Intake control for Groundtake
    controller
      .leftTrigger()
      .onTrue(
          new InstantCommand(() -> extendPivot())
              .andThen(new InstantCommand(() -> floorAlgaeIntake())))
      .onFalse(
          new InstantCommand(() -> retractPivot())
              .andThen(new InstantCommand(() -> keepAlgae())));

    controller
      .rightTrigger()
      .onTrue(new InstantCommand(() -> floorAlgaeOuttake()))
      .onFalse(new InstantCommand(() -> floorAlgaeStop()));

    // * -- Bindings for the button box --

    // | Elevator controls
    appendageJoystick.button(6)
      .onTrue(new InstantCommand(() -> l1()))
      .onFalse(score());

  appendageJoystick.button(5)
      .onTrue(new InstantCommand(() -> l2()))
      .onFalse(score());

    appendageJoystick.button(4)
      .onTrue(new InstantCommand(() -> l3()))
      .onFalse(score());

    appendageJoystick.button(3)
      .onTrue(new InstantCommand(() -> l4()))
      .onFalse(score());

    appendageJoystick.button(9).onTrue(new InstantCommand(() -> stow()));

    appendageJoystick.button(12).onTrue(intake());

    // | Dereefing controls
    appendageJoystick.button(8).onTrue(a1()).onFalse(retractAlgae());

    appendageJoystick.button(7).onTrue(a2()).onFalse(retractAlgae());

    // | Barge Scoring
    appendageJoystick
      .button(10)
      .onTrue(new InstantCommand(() -> l4()))
      .onFalse(actions.scoreBarge());

    appendageJoystick
      .button(11)
      .onTrue(new InstantCommand(() -> algaeOuttake()))
      .onFalse(new InstantCommand(() -> algaeStop()));

    // | Climber controls

    // | Full auto climbing
    // appendageJoystick.button(1)
    //     .onTrue(climber.autoClimb())

    // | Semi-automatic climbing
    // appendageJoystick.button(1)
    //     .onTrue(climber.extendClimber());

    // appendageJoystick.button(2)
    //     .onTrue(climber.retractClimber());

    // | Manual climbing
    appendageJoystick
      .button(1)
      .onTrue(new InstantCommand(() -> climbUp()))
      .onFalse(new InstantCommand(() -> stopClimb()));

    appendageJoystick
      .button(2)
      .onTrue(new InstantCommand(() -> climbDown()))
      .onFalse(new InstantCommand(() -> stopClimb()));
  
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

    public void poseEstimation() {

      var visionElevatorEst = visionElevator.getEstimatedGlobalPose(visionElevator.getCamera());
      var visionClimberEst = visionClimber.getEstimatedGlobalPose(visionClimber.getCamera());

      visionElevatorEst.ifPresent(
        est -> {
          var estStdDevs = visionElevator.getEstimationStdDevs();
          DogLog.log("Vision/Elevator Estimated Pose", est.estimatedPose);
          drivetrain.addVisionMeasurement(
              est.estimatedPose.toPose2d(),
              Utils.fpgaToCurrentTime(est.timestampSeconds),
              estStdDevs);
        });

      visionClimberEst.ifPresent(
        est -> {
          var estStdDevs = visionClimber.getEstimationStdDevs();
          DogLog.log("Vision/Climber Estimated Pose", est.estimatedPose);
          drivetrain.addVisionMeasurement(
              est.estimatedPose.toPose2d(),
              Utils.fpgaToCurrentTime(est.timestampSeconds),
              estStdDevs);
        });
    }

    public Command autoAim() {
      return Commands.sequence(
        drivetrain.resetAutoAimPID(), 
        drivetrain.goToPose(
          () -> drivetrain.getNearestReef()));
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

  public void getNearestReef() {
    drivetrain.getNearestReef();
  }

  public void setMarkers() {
    NamedCommands
      .registerCommand("Prep L4", new InstantCommand(() -> l4()));
  }
}
