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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Robot extends TimedRobot {

  private double maxSpeed = DriveConstants.maxSpeed;
  private double maxAngularRate = DriveConstants.maxAngularRate;
  private double slowSpeed = DriveConstants.slowSpeed;
  private double slowAngularRate = DriveConstants.slowAngularRate;

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

  // | | Subsystem Objects
  public final LED leds = new LED();
  public final Swerve drivetrain = DriveConstants.createDrivetrain();
  public final Vision visionElevator =
      new Vision(
          Constants.VisionConstants.elevatorCam, Constants.VisionConstants.elevatorCamOffset);
  public final Vision visionClimber =
      new Vision(Constants.VisionConstants.climberCam, Constants.VisionConstants.climberCamOffset);

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
    controller.leftBumper().onTrue(drivetrain.selectReef("Left"));
    controller.rightBumper().onTrue(drivetrain.selectReef("Right"));

    controller.y().onTrue(drivetrain.selectPiece("Coral"));
    controller.a().onTrue(drivetrain.selectPiece("Algae"));

    controller.x().whileTrue(autoAim());

    // * -- Bindings for the button box --

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
