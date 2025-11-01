package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.States.ArmStates;
import frc.robot.subsystems.States.ElevatorStates;
import frc.robot.subsystems.States.WristStates;

@SuppressWarnings("unused")
public class Robot extends TimedRobot {

  // | Bugs 
  // Bug - Algae detection needs retuning, boolean not updating so logic does not work

  private double maxSpeed = DriveConstants.maxSpeed;
  private double maxAngularRate = DriveConstants.maxAngularRate;
  private double slowSpeed = DriveConstants.slowSpeed;
  private double slowAngularRate = DriveConstants.slowAngularRate;

  private final Arm arm = new Arm();
  private final Elevator elevator = new Elevator();
  private final Wrist wrist = new Wrist();

  private final Intake intake = new Intake();

  boolean coralReady = false;
  boolean bargeReady = false;
  boolean processorReady = false;

  ElevatorStates scoringLevel = null;

  public double previousTimeStamp = Timer.getFPGATimestamp();

  private SendableChooser<AutoRoutine> autoChooser = new SendableChooser<>();

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

  public final LED leds = new LED();
  public final Swerve drivetrain = DriveConstants.createDrivetrain();
  public final Vision leftCamera =
      new Vision(
          Constants.VisionConstants.leftCam, Constants.VisionConstants.leftCameraOffset);
  public final Vision rightCamera =
      new Vision(Constants.VisionConstants.rightCam, Constants.VisionConstants.rightCameraOffset);

  // | Controller Objects
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick appendageJoystick = new CommandJoystick(1);

  // | Auton
  private final AutoFactory autoFactory;

  // * Create Mode Selector for single driver
  private enum driverMode {
    CORAL,
    ALGAE
  }

  private driverMode currentMode;

  public Robot() {

    // | Create Choreo
    autoFactory =
        new AutoFactory(
            () -> drivetrain.getState().Pose,
            drivetrain::resetPose,
            drivetrain::followTrajectory,
            false,
            drivetrain);

    autoChooser.addOption("Mid", mid());

    autoFactory.bind("Place L1", autoL1());

    // * -- Constant Bindings --
    // Not effected by driving mode
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(getThrottle())
                    .withVelocityY(getStrafe())
                    .withRotationalRate(getRotation())));
    
    controller
        .rightTrigger()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    driveSlow
                        .withVelocityX(getThrottle())
                        .withVelocityY(getStrafe())
                        .withRotationalRate(getRotation())));

    controller
        .leftTrigger()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    driveSlow
                        .withVelocityX(getThrottle())
                        .withVelocityY(getStrafe())
                        .withRotationalRate(getRotation())));

    resetGyro().onTrue(new InstantCommand(() -> drivetrain.seedFieldCentric()));

    // * Controller

    //  | Vision
    // controller.rightBumper().whileTrue(allignRight());
    // controller.leftBumper().whileTrue(allignLeft());

    stow().onTrue(reset());

    // | Intake
    wantToIntakeCoral()
        .onTrue(intakeCoral())
        .onFalse(prepCoral());

    wantToIntakeAlgae()
        .onTrue(intakeAlgae())
        .onFalse(prepAlgae());  
        
    // | Score
    wantToScore()
        .onTrue(place())
        .onFalse(score());

    // * Button Box

    // | Prep
    l4().onTrue(prepL4());

    l3().onTrue(prepMidPlace(ElevatorStates.L3));
    l2().onTrue(prepMidPlace(ElevatorStates.L2));

    l1().onTrue(prepTrough())
        .onFalse(trough());

    // | De-Reef
    deReefA2()
        .onTrue(grabAlgaeReef(ElevatorStates.ALGAE_L2))
        .onFalse(prepAlgae());
    deReefA1()
        .onTrue(grabAlgaeReef(ElevatorStates.ALGAE_L1))
        .onFalse(prepAlgae());

    // | Barge
    barge()
        .onTrue(prepBarge())
        .onFalse(scoreBarge());

    wantToProcess()
        .onTrue(prepProcess())
        .onFalse(process());

    // * Mode Switcher
    switchToCoral().onTrue(Commands.runOnce(() -> currentMode = driverMode.CORAL));
    switchToAlgae().onTrue(Commands.runOnce(() -> currentMode = driverMode.ALGAE));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void autonomousInit() {
    autoChooser.getSelected().cmd().schedule();
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    reset();
  }

  @Override
  public void robotInit(){
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  public void poseEstimation() {

    var visionLeftEst = leftCamera.getEstimatedGlobalPose(leftCamera.getCamera());
    var visionRightEst = rightCamera.getEstimatedGlobalPose(rightCamera.getCamera());

    visionLeftEst.ifPresent(
        est -> {
          var estStdDevs = leftCamera.getEstimationStdDevs();
          DogLog.log("Vision/LeftCam Estimated Pose", est.estimatedPose);
          drivetrain.addVisionMeasurement(
              est.estimatedPose.toPose2d(),
              Utils.fpgaToCurrentTime(est.timestampSeconds),
              estStdDevs);
        });

    visionRightEst.ifPresent(
        est -> {
          var estStdDevs = rightCamera.getEstimationStdDevs();
          DogLog.log("Vision/RightCam Estimated Pose", est.estimatedPose);
          drivetrain.addVisionMeasurement(
              est.estimatedPose.toPose2d(),
              Utils.fpgaToCurrentTime(est.timestampSeconds),
              estStdDevs);
        });
  }

  @Override
  public void disabledInit() {}

  @Override
  public void robotPeriodic() {

    DogLog.log("Loop Time", Timer.getFPGATimestamp() - previousTimeStamp);
    previousTimeStamp = Timer.getFPGATimestamp();

    if (hasCoral()) {
        leds.setPattern(leds.white);
    } else if (hasAlgae()) {
        leds.setPattern(leds.cyan);
    } else {
        leds.setPattern(leds.red);
    }

    CommandScheduler.getInstance().run();

    DogLog.setEnabled(true);

    // poseEstimation();

    // Log Logic
    DogLog.log("Has Coral", hasCoral());
    DogLog.log("Has Algae", hasAlgae());
            
    DogLog.log("Coral Is Ready", coralReady);
    DogLog.log("Barge Is Ready", bargeReady);
    DogLog.log("Process Is Ready", processorReady);
                        
    // Update Piece
    hasAlgae();
    hasCoral();

    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
    drivetrain.getNearestReef();
  }

  public Command autoAim() {
    return Commands.sequence(
        drivetrain.resetAutoAimPID(), 
        drivetrain.goToPose(
          () -> drivetrain.getNearestReef()));
  }

  // * AUTOMATION ACTIONS
  // was moved here becuase it didnt work in Automation class :(

    // * Reset / Defualt
    public Command reset() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.stop()), 

            Commands.runOnce(() -> elevator.setClampedGoal(ElevatorStates.STOW)),
            Commands.runOnce(() -> arm.setClampedGoal(ArmStates.STOW)),
            Commands.runOnce(() -> wrist.setClampedGoal(WristStates.CORAL)),

            Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll())
        );
    }

    // * Coral 

    // | L2 ^
    public Command place() {
        return Commands.sequence(
            Commands.runOnce(() -> arm.setClampedGoal(ArmStates.PLACE_CORAL))
        );
    }

    public Command score() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.placeCoral()),
            Commands.waitSeconds(0.5),
            reset()
        );
    }

    // | L1
    public Command prepTrough() {
        return Commands.sequence(
            Commands.runOnce(() -> elevator.setClampedGoal(ElevatorStates.L1)),
            Commands.runOnce(() -> arm.setClampedGoal(ArmStates.TROUGH)),
            Commands.runOnce(() -> wrist.setClampedGoal(WristStates.TROUGH)),
            Commands.runOnce(() -> coralReady = true)
        );
    }

    public Command trough() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.trough()),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> intake.stop()),
            reset()
        );
    }   

    // | Intake
    public Command intakeCoral() {
        // if (!hasCoral()) {
            return Commands.sequence(
                Commands.runOnce(() -> wrist.setClampedGoal(WristStates.INTAKE)),
                Commands.runOnce(() -> elevator.setClampedGoal(ElevatorStates.INTAKE)),
                Commands.runOnce(() -> arm.setClampedGoal(ArmStates.INTAKE)),
                grabCoral()
            );
        // } else {
        //     return reset();
        // }
    }

    public Command grabCoral() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.intakeCoral()),
            Commands.waitUntil(() -> hasCoral()),
            Commands.runOnce(() -> intake.stop())
        );
    }

    // * Algae

    // | De-Reef
    public Command grabAlgaeReef(ElevatorStates level) {
        return Commands.sequence(
            Commands.runOnce(() -> wrist.setClampedGoal(WristStates.ALGAE)),
            Commands.runOnce(() -> elevator.setClampedGoal(level)),
            Commands.runOnce(() -> arm.setClampedGoal(ArmStates.REMOVE_ALGAE)),
            grabAlgae()
        );
    }

    // | Process
    public Command prepProcess() {
        return Commands.sequence(
            Commands.runOnce(() -> elevator.setClampedGoal(ElevatorStates.STOW)),
            Commands.runOnce(() -> arm.setClampedGoal(ArmStates.PROCESS)),
            Commands.runOnce(() -> wrist.setClampedGoal(WristStates.PROCESS)),
            Commands.runOnce(() -> processorReady = true)
        );
    }

    public Command process() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.outtake()),
            Commands.waitSeconds(1),
            reset()
        );
    }

    // | Barge
    public Command scoreBarge() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.outtake()),
            Commands.waitSeconds(0.5),
            reset()
        );
    }

    // | Intake
    public Command intakeAlgae() {
        // if (getPiece() == '!') {
            return Commands.sequence(
                Commands.runOnce(() -> wrist.setClampedGoal(WristStates.INTAKE)),
                Commands.runOnce(() -> elevator.setClampedGoal(ElevatorStates.ALGAE_INTAKE)),
                Commands.runOnce(() -> arm.setClampedGoal(ArmStates.INTAKE_ALGAE)),
                grabAlgae()
            );
        // } else {
        //     return reset();
        // }
    }

    public Command grabAlgae() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.intakeAlgae()),
            Commands.waitUntil(() -> hasAlgae()),
            Commands.runOnce(() -> intake.hold())
        );
    }

    // * Prep

    // | Coral

    // | Used for L2 & l3
    public Command prepMidPlace(ElevatorStates level) {
        return Commands.sequence(
            Commands.runOnce(() -> wrist.setClampedGoal(WristStates.CORAL)),
            Commands.runOnce(() -> elevator.setClampedGoal(level)),
            Commands.runOnce(() -> arm.setClampedGoal(ArmStates.PREP_CORAL)),
            Commands.waitUntil(() -> elevator.atSetpoint()),
            Commands.waitUntil(() -> arm.atSetpoint()),
            Commands.runOnce(() -> coralReady = true)
        );
    }

    public Command prepL4() {
        return Commands.sequence(
            Commands.runOnce(() -> wrist.setClampedGoal(WristStates.CORAL)),
            Commands.runOnce(() -> elevator.setClampedGoal(ElevatorStates.L4)),
            Commands.runOnce(() -> arm.setClampedGoal(ArmStates.PREP_L4)),
            Commands.waitUntil(() -> elevator.atSetpoint()),
            Commands.waitUntil(() -> arm.atSetpoint()),
            Commands.runOnce(() -> coralReady = true)
        );
    }

    public Command prepCoral() {
        if (hasCoral()) {
            return Commands.sequence(
                Commands.runOnce(() -> arm.setClampedGoal(ArmStates.HOLD_CORAL)),
                Commands.runOnce(() -> wrist.setClampedGoal(WristStates.CORAL))
            );
        } else {
            return reset();
        }
    }

    // | Algae
    public Command prepAlgae() {
        // if (hasAlgae()) {
            return Commands.sequence(
                Commands.runOnce(() -> elevator.setClampedGoal(ElevatorStates.ALGAE_STOW)),
                Commands.runOnce(() -> arm.setClampedGoal(ArmStates.STOW)),
                Commands.runOnce(() -> wrist.setClampedGoal(WristStates.ALGAE))
            );
        // } else {
        //     return reset();
        // }
    }

    public Command prepBarge() {
        // if (getPiece() == 'A') {
            return Commands.sequence(
                Commands.runOnce(() -> wrist.setClampedGoal(WristStates.ALGAE)),
                Commands.runOnce(() -> elevator.setClampedGoal(ElevatorStates.BARGE)),
                Commands.runOnce(() -> arm.setClampedGoal(ArmStates.BARGE)),
                Commands.runOnce(() -> bargeReady = true)
            );
        // } else {
        //     return reset();
        // }
    }

    // * Logic

    private boolean hasCoral() {
        if (intake.getObjectDistance() <= 0.04) {
            if (intake.getR() >= 0 && intake.getR() <= 0.06 &&
                intake.getG() >= 0 && intake.getG() <= 0.06 && 
                intake.getB() >= 0 && intake.getB() <= 0.06) {
                    
                return true;
            } else {
                return false;
            }
        } else {
            return false;            
        }
    }

    // fix-me
    private boolean hasAlgae() {
        if (intake.getObjectDistance() <= 0.05) {
            if (intake.getR() >= 0.07 && intake.getR() <= 0.13 && 
                intake.getG() >= 0.27 && intake.getG() <= 0.34 && 
                intake.getB() >= 0.09 && intake.getB() <= 0.15) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;            
        }
    }

    // Use if needed to prep coral before spearing
    // private boolean coralPreped() {}

    // * Parallel

    private Command setPlace() {
        return Commands.runOnce(() -> arm.setClampedGoal(ArmStates.PLACE_CORAL));
    }

    //  * Define triggers here

  // Swerve
  // Use exponential joystick for more acceleration control
  // Linear: input = output
  // Exponential: greater input = greater output
  private double getThrottle() {
    return -(Math.pow(Math.abs(controller.getLeftY()), 1.2)) * Math.signum(controller.getLeftY());
  }

  private double getStrafe() {
    return -(Math.pow(Math.abs(controller.getLeftX()), 1.2)) * Math.signum(controller.getLeftX());
  }

  private double getRotation() {
    return -(Math.pow(Math.abs(controller.getRightX()), 1.5)) * Math.signum(controller.getRightX());
  }

  // Gyro
  private Trigger resetGyro() {
    return controller.rightStick().and(controller.leftStick());
  }

  // Superstructure
  private Trigger stow() {
    return controller.povDown();
  }

  // Mode Switcher
  private Trigger switchToCoral() {
    return controller.start();
  }

  private Trigger switchToAlgae() {
    return controller.back();
  }

  //  --- CORAL ---
  private Trigger wantToIntakeCoral() {
    if (currentMode == driverMode.CORAL) {
      return controller.leftTrigger();
    } else {
      return null;
    }
  }

  private Trigger l1() {
    if (currentMode == driverMode.CORAL) {
      return controller.b();
    } else {
      return null;
    }
  }

  private Trigger l2() {
    if (currentMode == driverMode.CORAL) {
      return controller.a();
    } else {
      return null;
    }
  }

  private Trigger l3() {
    if (currentMode == driverMode.CORAL) {
      return controller.x();
    } else {
      return null;
    }
  }

  private Trigger l4() {
    if (currentMode == driverMode.CORAL) {
      return controller.y();
    } else {
      return null;
    }
  }

  // Score method in superstructure will need logic to determoine if we score on mid or l4, because they 
  // will have differnt scoring methods due to differnt branch shapes
  private Trigger wantToScore() {
    if (currentMode == driverMode.CORAL) {
      return controller.rightTrigger();
    } else {
      return null;
    }
  }

  // --- ALGAE ---
  private Trigger wantToIntakeAlgae() {
    if (currentMode == driverMode.ALGAE) {
      return controller.leftTrigger();
    } else {
      return null;
    }
  }

  private Trigger deReefA1() {
    if (currentMode == driverMode.ALGAE) {
      return controller.a();
    } else {
      return null;
    }
  }

  private Trigger deReefA2() {
    if (currentMode == driverMode.ALGAE) {
      return controller.y();
    } else {
      return null;
    }
  }

  private Trigger wantToProcess() {
    if (currentMode == driverMode.ALGAE) {
      return controller.b();
    } else {
      return null;
    }
  }

  private Trigger barge() {
    if (currentMode == driverMode.ALGAE) {
      return controller.x();
    } else {
      return null;
    }
  }

  // Vision - Implement after IO if fully working
  private Trigger allignLeft() {
    if (currentMode == driverMode.CORAL) {
      return controller.leftBumper();
    } else {
      return null;
    }
  }

  private Trigger allignRight() {
    if (currentMode == driverMode.CORAL) {
      return controller.rightBumper();
    } else {
      return null;
    }
  }

  private Trigger allignAlgae() {
    if (currentMode == driverMode.ALGAE) {
      return controller.leftBumper();
    } else {
      return null;
    }
  }
    
    // ----------
    
    // ! TESTING ONLY COMMANDS

    public void vertical() {
        wrist.setClampedGoal(WristStates.CORAL);
    }

    public void horizontal() {
        wrist.setClampedGoal(WristStates.ALGAE);
    }

    public void back() {
        arm.setClampedGoal(ArmStates.BACK);
    }

    public void front() {
        arm.setClampedGoal(ArmStates.FRONT);
    }

    public void middle() {
        arm.setClampedGoal(ArmStates.MIDDLE);
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

    // | Auto Commands
    public Command autoL1() {
        return Commands.sequence(
            Commands.runOnce(() -> prepTrough()),
            Commands.waitUntil(() -> arm.atSetpoint()),
            Commands.runOnce(() -> trough())
        );
    }
    
    // | Auto paths
    public AutoRoutine mid() {
        AutoRoutine routine = autoFactory.newRoutine("L1 Straight");
        AutoTrajectory trajectory = routine.trajectory("L1 Straight");

        routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
        return routine;
    }
}
