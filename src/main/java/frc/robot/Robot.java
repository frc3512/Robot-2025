package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Groundtake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Reeftake;
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
          .withRotationalDeadband(maxAngularRate * 0.07) // Add a 7% deadband
          .withDriveRequestType(DriveRequestType.Velocity);

  private final SwerveRequest.FieldCentric driveSlow =
      new SwerveRequest.FieldCentric()
          .withDeadband(slowSpeed * 0.1)
          .withRotationalDeadband(slowAngularRate * 0.07) // Add a 7% deadband
          .withDriveRequestType(DriveRequestType.Velocity);

  // Subsystem Objects
  public final Climber climber = new Climber();
  public final Elevator elevator = new Elevator();
  public final Groundtake groundtake = new Groundtake();
  public final LED leds = new LED();
  public final Reeftake reeftake = new Reeftake();
  public final Swerve drivetrain = DriveConstants.createDrivetrain();
  public final Vision vision = new Vision();

  // Controller Objects
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick appendageJoystick = new CommandJoystick(1);

  // Driver Camera Thread for crosshair
  private final Thread m_visionThread;

  // Auton
  private final AutoFactory autoFactory;

  public Robot() {

    CameraServer.startAutomaticCapture();

    m_visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(640, 480);

              CvSink cvSink = CameraServer.getVideo();
              CvSource outputStream = CameraServer.putVideo("Drive Cam", 640, 480);

              Mat mat = new Mat();
              Point pt1 = new Point(0, 65);
              Point pt2 = new Point(400, 65);
              Point pt3 = new Point(0, 55);
              Point pt4 = new Point(400, 55);
              Scalar color = new Scalar(28, 239, 84);

              while (!Thread.interrupted()) {

                if (cvSink.grabFrame(mat) == 0) {
                  outputStream.notifyError(cvSink.getError());
                  continue;
                }

                Imgproc.line(mat, pt1, pt2, color, 2);
                Imgproc.line(mat, pt3, pt4, color, 2);
                outputStream.putFrame(mat);
              }
            });

    m_visionThread.setDaemon(true);
    m_visionThread.start();

    // Create Choreo
    autoFactory =
        new AutoFactory(
            () -> drivetrain.getState().Pose,
            drivetrain::resetPose,
            drivetrain::followTrajectory,
            false,
            drivetrain);

    autoFactory
        .bind("Score l4", scorel4())
        .bind("Intake", intake());

    autoChooser
        .addOption("Mid l4", midl4());

    // Controler Bindings
    controller.rightBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    driveSlow
                        .withVelocityX(-controller.getLeftY() * slowSpeed)
                        .withVelocityY(-controller.getLeftX() * slowSpeed)
                        .withRotationalRate(-controller.getRightX() * slowAngularRate)))
        .whileFalse(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(-controller.getLeftY() * maxSpeed)
                        .withVelocityY(-controller.getLeftX() * maxSpeed)
                        .withRotationalRate(-controller.getRightX() * maxAngularRate)));

    controller.x()
        .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    
    // Intake control for Groundtake
    controller
        .leftTrigger()
        .onTrue(
            new InstantCommand(() -> groundtake.extendPivot())
                .andThen(new InstantCommand(() -> groundtake.floorAlgaeIntake())))
        .onFalse(
            new InstantCommand(() -> groundtake.retractPivot())
                .andThen(new InstantCommand(() -> groundtake.keepAlgae())));

    controller
        .rightTrigger()
        .onTrue(
            new InstantCommand(() -> groundtake.floorAlgaeOuttake()))
        .onFalse(
            new InstantCommand(() -> groundtake.floorAlgaeStop()));

    // Bindings for the button box

    // Intake control for Groundtake
    controller
        .leftTrigger()
        .onTrue(
            new InstantCommand(() -> groundtake.extendPivot())
                .andThen(new InstantCommand(() -> groundtake.floorAlgaeIntake())));
    controller
        .leftTrigger()
        .onFalse(
            new InstantCommand(() -> groundtake.retractPivot())
                .andThen(new InstantCommand(() -> groundtake.keepAlgae())));

    controller.rightTrigger().onTrue(new InstantCommand(() -> groundtake.floorAlgaeOuttake()));
    controller.rightTrigger().onFalse(new InstantCommand(() -> groundtake.floorAlgaeStop()));

    // Dereefing controls
    appendageJoystick.button(8)
        .onTrue(a1())
        .onFalse(retractAlgae());

    appendageJoystick.button(7)
        .onTrue(a2())
        .onFalse(retractAlgae());

    appendageJoystick.button(9)
      . onTrue(new InstantCommand(() -> elevator.stow()));

    // Barge Scoring
    appendageJoystick.button(10)
        .onTrue(new InstantCommand(() -> elevator.l4()))
        .onFalse(scoreBarge());

    // Elevator controls
    appendageJoystick.button(6)
        .onTrue(new InstantCommand(() -> elevator.l1()))
        .onTrue(leds.runPattern(leds.blue))
        .onFalse(score());

    appendageJoystick.button(5)
        .onTrue(new InstantCommand(() -> elevator.l2()))
        .onTrue(leds.runPattern(leds.blue))
        .onFalse(score());

    appendageJoystick.button(4)
        .onTrue(new InstantCommand(() -> elevator.l3()))
        .onTrue(leds.runPattern(leds.blue))
        .onFalse(score());

    appendageJoystick.button(3)
        .onTrue(new InstantCommand(() -> elevator.l4()))
        .onTrue(leds.runPattern(leds.blue))
        .onFalse(score());

    appendageJoystick.button(9)
        .onTrue(new InstantCommand(() -> elevator.stow()));

    appendageJoystick.button(12)
        .onTrue(intake());

    // Climber controls
    appendageJoystick.button(1)
        .onTrue(climber.setClimber(0.8))
        .onFalse(climber.setClimber(0.0));

    appendageJoystick.button(2)
        .onTrue(climber.setClimber(-0.8))
        .onFalse(climber.setClimber(0.0));

    SmartDashboard
        .putData("Auto Chooser", autoChooser);
  }

  @Override
  public void autonomousInit() {
    autoChooser
        .getSelected()
        .cmd()
        .schedule();
  }

  @Override
  public void teleopInit() {
    elevator.setClampedGoal(Constants.ElevatorConstants.stowPos);
    groundtake.setGoal(Constants.GroundtakeConstants.stowPos);

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
                    () ->
                        drive
                            .withVelocityX(-controller.getLeftY() * maxSpeed)
                            .withVelocityY(-controller.getLeftX() * maxSpeed)
                            .withRotationalRate(-controller.getRightX() * maxAngularRate)));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void robotPeriodic() {
    vision.getPose();

    var visionEst = vision.getEstimatedGlobalPose();
    visionEst.ifPresent(
            est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = vision.getEstimationStdDevs();

                drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });

    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {}

  public SequentialCommandGroup a1() {
    return new InstantCommand(() -> elevator.a1())
        .andThen(new InstantCommand(() -> reeftake.algaeIntake()));
  }

  public SequentialCommandGroup a2() {
    return new InstantCommand(() -> elevator.a2())
        .andThen(new InstantCommand(() -> reeftake.algaeIntake())
        );
  }

  public SequentialCommandGroup scoreBarge() {
    return new InstantCommand(() -> reeftake.algaeOuttake())
        .andThen(new WaitCommand(0.75))
        .andThen(new InstantCommand(() -> elevator.hp()))
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
    return new InstantCommand(() -> elevator.hp())
        .andThen(reeftake.autoIntake())
        .andThen(leds.runPattern(leds.scrollngRainbow));
  }

  public SequentialCommandGroup scorel4() {
    return new InstantCommand(() -> elevator.l4())
     .andThen(new WaitCommand(1.5))
     .andThen(score());
  }

  // Auto paths
  public AutoRoutine midl4() {

    AutoRoutine routine = autoFactory.newRoutine("Mid l4");
    AutoTrajectory trajectory = routine.trajectory("Mid l4");

    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
    return routine;
  }
}
