package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.States.ArmStates;
import frc.robot.subsystems.States.ElevatorStates;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@SuppressWarnings("unused")
public class Robot extends TimedRobot {

  // | Bugs 

  // Fix: Automation after it has been moved into Robot.java

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

  char piece = 'C';

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

  // | Driver Camera Thread for crosshair
  private final Thread m_visionThread;

  // | Auton
  private final AutoFactory autoFactory;

  public Robot() {

    CameraServer.startAutomaticCapture();

    m_visionThread =
        new Thread(
            () -> {
              // * Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // * Set the resolution
              camera.setResolution(320, 240);

              CvSink cvSink = CameraServer.getVideo();
              CvSource outputStream = CameraServer.putVideo("Drive Cam", 640, 480);

              Mat mat = new Mat();
              Point pt1 = new Point(0, 65);
              Point pt2 = new Point(400, 65);
              Point pt3 = new Point(0, 55);
              Point pt4 = new Point(400, 55);
              Point pt5 = new Point(0, 30);
              Point pt6 = new Point(400, 30);
              Scalar coralColor = new Scalar(28, 239, 84);
              Scalar algaeColor = new Scalar(18, 5, 92);

              while (!Thread.interrupted()) {

                if (cvSink.grabFrame(mat) == 0) {
                  outputStream.notifyError(cvSink.getError());
                  continue;
                }

                Imgproc.line(mat, pt1, pt2, coralColor, 2);
                Imgproc.line(mat, pt3, pt4, coralColor, 2);
                Imgproc.line(mat, pt5, pt6, algaeColor, 3);
                outputStream.putFrame(mat);
              }
            });

    m_visionThread.setDaemon(true);
    m_visionThread.start();

    // | Create Choreo
    autoFactory =
        new AutoFactory(
            () -> drivetrain.getState().Pose,
            drivetrain::resetPose,
            drivetrain::followTrajectory,
            false,
            drivetrain);

    autoChooser.addOption("Mid l4", midl4());
    autoChooser.addOption("Mid l4 - Barge", midl4Barge());

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
    // controller.leftBumper().onTrue(drivetrain.selectReef("Left"));
    // controller.rightBumper().onTrue(drivetrain.selectReef("Right"));

    // controller.y().onTrue(drivetrain.selectPiece("Coral"));
    // controller.a().onTrue(drivetrain.selectPiece("Algae"));

    // controller.x().whileTrue(autoAim());

    // * -- Bindings for the button box --

    // ! TESTING BINDS

    // controller.button(8).onTrue(new InstantCommand(() -> setCoralMode()));
    // controller.button(7).onTrue(new InstantCommand(() -> setAlgaeMode()));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void autonomousInit() {
    autoChooser.getSelected().cmd().schedule();
  }

  @Override
  public void teleopInit() {}

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

    CommandScheduler.getInstance().run();
    // | Tuning mode
    DogLog.setEnabled(true);

    poseEstimation();

    // Log Logic
    DogLog.log("Has Coral", hasCoral());
    DogLog.log("Has Algae", hasAlgae());
            
    DogLog.log("Coral Is Ready", coralReady);
    DogLog.log("Barge Is Ready", bargeReady);
    DogLog.log("Process Is Ready", processorReady);
                        
    // Update Piece
    hasAlgae();
    hasCoral();

    setTestMode();
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

  public void setCoralMode() {

    drivetrain.selectPiece("Coral");

    controller.rightBumper().whileTrue(allignRight());
    controller.leftBumper().whileTrue(allignLeft());

    // Prep Score
    controller.y()
        .onTrue(prepScore(ElevatorStates.L4));
    controller.x()
        .onTrue(prepScore(ElevatorStates.L3));
    controller.a()
        .onTrue(prepScore(ElevatorStates.L2));

    // Score
    controller.b()
        .onTrue(score())
        .onFalse(reset());

    // | Manual Feed
    controller.leftTrigger()
        .onTrue(new InstantCommand(() -> intake()))
        .onTrue(new InstantCommand(() -> back()))
        .onFalse(new InstantCommand(() -> stopRollers()))
        .onFalse(new InstantCommand(() -> middle()));
        
    controller.rightTrigger()
        .onTrue(reset());

    // | Ground Intake
    // controller.rightTrigger()
    //     .onTrue(intakeCoral())
    //     .onFalse(reset());
  }

  public void setAlgaeMode() {

    drivetrain.selectPiece("Algae");

    controller.rightBumper().whileTrue(allignAlgae());

  }

  public void setTestMode() {

    controller.a().onTrue(new InstantCommand(() -> back()));
    controller.y().onTrue(new InstantCommand(() -> front()));
    controller.x().onTrue(new InstantCommand(() -> middle()));

  }

  // * AUTOMATION ACTIONS
  // was moved here becuase it didnt work in Automation class :(

    public Command allignLeft() {
        return Commands.sequence(
            drivetrain.selectReef("Left"),
            drivetrain.resetAutoAimPID(), 
            drivetrain.goToPose(
                () -> drivetrain.getNearestReef())
        );
    }

    public Command allignRight() {
        return Commands.sequence(
            drivetrain.selectReef("Right"),
            drivetrain.resetAutoAimPID(), 
            drivetrain.goToPose(
                () -> drivetrain.getNearestReef())
        );
    }

    public Command allignAlgae() {
        return Commands.sequence(
            drivetrain.selectPiece("Algae"),
            drivetrain.resetAutoAimPID(), 
            drivetrain.goToPose(
                () -> drivetrain.getNearestReef())
        );
    }

  // * Reset / Defualt
    public Command reset() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.stop()), 
            Commands.runOnce(() -> elevator.setClampedGoal(ElevatorStates.STOW)),
            Commands.runOnce(() -> arm.setClampedGoal(ArmStates.STOW)),
            // Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.vertical)),
            Commands.runOnce(() -> coralReady = false),
            Commands.runOnce(() -> bargeReady = false),
            Commands.runOnce(() -> processorReady = false)
        );
    }

    public void stow() {
        elevator.setClampedGoal(ElevatorStates.STOW);
        arm.setClampedGoal(ArmStates.STOW);
        intake.stop();
    }

    // * Full Auto

    // | Coral
    public Command autoScore(ElevatorStates pos) {
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
            Commands.parallel(setPlace(), placeCoral())
        );
    }

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
    public Command intakeCoral() {
        if (!hasCoral()) {
            return Commands.sequence(
                // Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.horizontal)),
                Commands.runOnce(() -> elevator.setClampedGoal(ElevatorStates.INTAKE)),
                Commands.runOnce(() -> arm.setClampedGoal(ArmStates.INTAKE)),
                Commands.waitUntil(() -> arm.atSetpoint()),
                // Commands.waitUntil(() -> wrist.atGoal()),
                grabCoral()
            );
        } else {
            return reset();
        }
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
            Commands.waitUntil(() -> hasAlgae()),
            Commands.runOnce(() -> intake.hold())
        );
    }

    // * Prep

    // | Coral
    public Command prepScore(ElevatorStates pos) {
        return Commands.sequence(
            // Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.vertical)),
            // Commands.waitUntil(() -> wrist.atGoal()),
            Commands.runOnce(() -> elevator.setClampedGoal(pos)),
            Commands.runOnce(() -> arm.setClampedGoal(ArmStates.PREP_CORAL)),
            Commands.waitSeconds(0.75),
            Commands.waitUntil(() -> elevator.atSetpoint()),
            Commands.waitUntil(() -> arm.atSetpoint()),
            Commands.runOnce(() -> coralReady = true)
        );
    }

    public void setLevel(ElevatorStates level) {
        elevator.setClampedGoal(level);
        arm.setClampedGoal(ArmStates.PREP_CORAL);
    }

    public Command prepCoral() {
        return Commands.sequence(
            Commands.runOnce(() -> arm.setClampedGoal(ArmStates.HOLD_CORAL))
            // Commands.runOnce(() -> wrist.setClampedGoal(Constants.WristConstants.vertical))
        );
    }

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
        return Commands.runOnce(() -> intake.placeCoral());
    }
    
    // ----------
    
    // ! TESTING ONLY COMMANDS


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
    
    // | Auto paths
    public AutoRoutine midl4() {
        AutoRoutine routine = autoFactory.newRoutine("Mid l4");
        AutoTrajectory trajectory = routine.trajectory("Mid l4");

        routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
        return routine;
    }

    public AutoRoutine midl4Barge() {
        AutoRoutine routine = autoFactory.newRoutine("Mid l4 - Barge");
        AutoTrajectory trajectory = routine.trajectory("Mid l4 - Barge");
        AutoTrajectory trajectory2 = routine.trajectory("De-reef");

        routine
            .active()
            .onTrue(
                Commands.sequence(
                    trajectory.resetOdometry(),
                    trajectory.cmd(),
                    trajectory2.resetOdometry(),
                    trajectory2.cmd()));
        return routine;
    }
}
