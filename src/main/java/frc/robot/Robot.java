package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

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

// import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {

  private double MaxSpeed = DriveConstants.MaxSpeed;
  private double MaxAngularRate = DriveConstants.MaxAngularRate;
  private SendableChooser<AutoRoutine> autoChooser = new SendableChooser<>();

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.07) // Add a 7% deadband
          .withDriveRequestType(DriveRequestType.Velocity);

  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  // Subsystem Objects
  public final Climber climber = new Climber();
  public final Elevator elevator = new Elevator();
  public final Groundtake groundtake = new Groundtake();
  public final LED leds = new LED();
  public final Reeftake reeftake = new Reeftake();
  public final Swerve drivetrain = DriveConstants.createDrivetrain();
  // public final Vision vision = new Vision();

  // Controller Objects
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick appendageJoystick = new CommandJoystick(1);

  //Driver camera thread crosshair
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
            true,
            drivetrain);

    autoChooser.addOption("Orbit Reef", orbitReef());

    // Controler Bindings
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-controller.getLeftY() * MaxSpeed)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed)
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate)));

    controller.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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

    // Reeftake controls
    appendageJoystick.button(10).onTrue(new InstantCommand(() -> reeftake.algaeIntake()));
    appendageJoystick.button(10).onFalse(new InstantCommand(() -> reeftake.algaeStop()));

    // Elevator controls
    appendageJoystick.button(6).onTrue(new InstantCommand(() -> elevator.l1()));
    appendageJoystick.button(6).onTrue(leds.runPattern(leds.blue));
    appendageJoystick.button(6).onFalse(score());
    appendageJoystick.button(6).onFalse(leds.runPattern(leds.red));

    appendageJoystick.button(5).onTrue(new InstantCommand(() -> elevator.l2()));
    appendageJoystick.button(5).onTrue(leds.runPattern(leds.blue));
    appendageJoystick.button(5).onFalse(score());
    appendageJoystick.button(5).onFalse(leds.runPattern(leds.red));

    appendageJoystick.button(4).onTrue(new InstantCommand(() -> elevator.l3()));
    appendageJoystick.button(4).onTrue(leds.runPattern(leds.blue));
    appendageJoystick.button(4).onFalse(score());
    appendageJoystick.button(4).onFalse(leds.runPattern(leds.red));

    appendageJoystick.button(3).onTrue(new InstantCommand(() -> elevator.l4()));
    appendageJoystick.button(3).onTrue(leds.runPattern(leds.blue));
    appendageJoystick.button(3).onFalse(score());
    appendageJoystick.button(3).onFalse(leds.runPattern(leds.red));

    appendageJoystick.button(8).onTrue(a1());
    appendageJoystick.button(8).onFalse(new InstantCommand(() -> reeftake.algaeStop()));

    appendageJoystick.button(7).onTrue(a2());
    appendageJoystick.button(7).onFalse(new InstantCommand(() -> reeftake.algaeStop()));

    appendageJoystick.button(9).onTrue(new InstantCommand(() -> elevator.stow()));

    appendageJoystick
        .button(12)
        .onTrue(
            new InstantCommand(() -> elevator.hp())
                .andThen(reeftake.reeftakeIntake())
                .andThen(leds.runPattern(leds.scrollngRainbow))
                .andThen(new InstantCommand(() -> elevator.stow())));

    // Manual control, COMMENT OUT WHEN NOT IN USE
    // DISABLE PID IN ELEVATOR CLASS!!!
    // appendageJoystick.button(7).onTrue(elevator.manualElevator(0.2));
    // appendageJoystick.button(7).onFalse(elevator.manualElevator(0.0));

    // appendageJoystick.button(8).onTrue(elevator.manualElevator(-0.1));
    // appendageJoystick.button(8).onFalse(elevator.manualElevator(0.0));

    // appendageJoystick.button(11).onTrue(new InstantCommand(() -> elevator.zeroMotor()));

    // Climber controls
    appendageJoystick.button(1).onTrue(new InstantCommand(() -> climber.setClimber(0.8)));
    appendageJoystick.button(1).onFalse(new InstantCommand(() -> climber.setClimber(0.0)));

    appendageJoystick.button(2).onTrue(new InstantCommand(() -> climber.setClimber(-0.8)));
    appendageJoystick.button(2).onFalse(new InstantCommand(() -> climber.setClimber(0.0)));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void autonomousInit() {
    autoChooser.getSelected().cmd().schedule();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledExit() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {}

  // Make sequencial commands for the elevator here once reeftake pivot it made.
  public SequentialCommandGroup a1() {
    return new InstantCommand(() -> elevator.a1())
        .andThen(new InstantCommand(() -> reeftake.algaeIntake()));
  }

  public SequentialCommandGroup a2() {
    return new InstantCommand(() -> elevator.a2())
        .andThen(new InstantCommand(() -> reeftake.algaeIntake())
        );
  }

  public SequentialCommandGroup score() {
    return new InstantCommand(() -> reeftake.coralIntake())
        .andThen(new WaitCommand(0.5))
        .andThen(new InstantCommand(() -> elevator.stow()))
        .andThen(new InstantCommand(() -> reeftake.coralStop()));
  }

  public SequentialCommandGroup scorel2() {
    return new InstantCommand(() -> elevator.l2())
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> reeftake.coralIntake()))
        .andThen(new WaitCommand(0.75))
        .andThen(
            new InstantCommand(() -> reeftake.coralStop())
                .andThen(new InstantCommand(() -> elevator.stow())));
  }

  public SequentialCommandGroup scorel3() {
    return new InstantCommand(() -> elevator.l3())
        .andThen(new WaitCommand(2))
        .andThen(new InstantCommand(() -> reeftake.coralIntake()))
        .andThen(new WaitCommand(0.75))
        .andThen(
            new InstantCommand(() -> reeftake.coralStop())
                .andThen(new InstantCommand(() -> elevator.stow())));
  }

  public SequentialCommandGroup scorel4() {
    return new InstantCommand(() -> elevator.l4())
        .andThen(new WaitCommand(2.5))
        .andThen(new InstantCommand(() -> reeftake.coralIntake()))
        .andThen(new WaitCommand(0.75))
        .andThen(
            new InstantCommand(() -> reeftake.coralStop())
                .andThen(new InstantCommand(() -> elevator.stow())));
  }

  public AutoRoutine orbitReef() {

    AutoRoutine routine = autoFactory.newRoutine("Orbit Reef");
    AutoTrajectory trajectory = routine.trajectory("Orbit Reef");

    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));

    return routine;
  }
}
