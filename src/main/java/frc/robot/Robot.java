package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

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
import frc.robot.Auton.Autos;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {

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
  
  public final Autos autos = new Autos(drivetrain);

  // | Driver Camera Thread for crosshair
  private final Thread m_visionThread;

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
            new InstantCommand(() -> actions.extendPivot())
                .andThen(new InstantCommand(() -> actions.floorAlgaeIntake())))
        .onFalse(
            new InstantCommand(() -> actions.retractPivot())
                .andThen(new InstantCommand(() -> actions.keepAlgae())));

    controller
        .rightTrigger()
        .onTrue(new InstantCommand(() -> actions.floorAlgaeOuttake()))
        .onFalse(new InstantCommand(() -> actions.floorAlgaeStop()));

    // * -- Bindings for the button box --

    // | Elevator controls
    appendageJoystick.button(6)
      .onTrue(new InstantCommand(() -> actions.l1()))
      .onFalse(actions.score());

    appendageJoystick.button(5)
      .onTrue(new InstantCommand(() -> actions.l2()))
      .onFalse(actions.score());

    appendageJoystick.button(4)
      .onTrue(new InstantCommand(() -> actions.l3()))
      .onFalse(actions.score());

    appendageJoystick.button(3)
      .onTrue(new InstantCommand(() -> actions.l4()))
      .onFalse(actions.score());

    appendageJoystick.button(9).onTrue(new InstantCommand(() -> actions.stow()));

    appendageJoystick.button(12).onTrue(actions.intake());

    // | Dereefing controls
    appendageJoystick.button(8).onTrue(actions.a1()).onFalse(actions.retractAlgae());

    appendageJoystick.button(7).onTrue(actions.a2()).onFalse(actions.retractAlgae());

    // | Barge Scoring
    appendageJoystick
        .button(10)
        .onTrue(new InstantCommand(() -> actions.l4()))
        .onFalse(actions.scoreBarge());

    appendageJoystick
        .button(11)
        .onTrue(new InstantCommand(() -> actions.algaeOuttake()))
        .onFalse(new InstantCommand(() -> actions.algaeStop()));

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
        .onTrue(new InstantCommand(() -> actions.climbUp()))
        .onFalse(new InstantCommand(() -> actions.stopClimb()));

    appendageJoystick
        .button(2)
        .onTrue(new InstantCommand(() -> actions.climbDown()))
        .onFalse(new InstantCommand(() -> actions.stopClimb()));
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void robotInit(){
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
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

  @Override
  public void disabledInit() {}

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
    // | Tuning mode
    DogLog.setEnabled(Constants.GeneralConstants.tuningMode);

    poseEstimation();
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
}
