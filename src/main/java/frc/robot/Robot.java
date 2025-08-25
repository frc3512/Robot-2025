package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@SuppressWarnings("unused")
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
  public final Swerve drivetrain = DriveConstants.createDrivetrain();
  public final LEDs leds = new LEDs();

  public final Vision visionLeft =
      new Vision(
          Constants.VisionConstants.leftCam, Constants.VisionConstants.leftCameraOffset);
  public final Vision visionRight =
      new Vision(
          Constants.VisionConstants.rightCam, Constants.VisionConstants.rightCameraOffset);

  // Controller Objects
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick appendageJoystick = new CommandJoystick(1);

  // Auton
  private final AutoFactory autoFactory;

  public Robot() {

    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));

    // Create Choreo
    autoFactory =
        new AutoFactory(
            () -> drivetrain.getState().Pose,
            drivetrain::resetPose,
            drivetrain::followTrajectory,
            false,
            drivetrain);

    autoChooser.addOption("Mid l4", midl4());

    // -- Controler Bindings --

    // Drivetrain control
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-controller.getLeftY() * maxSpeed)
                    .withVelocityY(-controller.getLeftX() * maxSpeed)
                    .withRotationalRate(-controller.getRightX() * maxAngularRate)));

    // Slow mode
    controller.leftBumper().whileTrue(
        drivetrain.applyRequest(
            () ->
                driveSlow
                    .withVelocityX(-controller.getLeftY() * slowSpeed)
                    .withVelocityY(-controller.getLeftX() * slowSpeed)
                    .withRotationalRate(-controller.getRightX() * slowAngularRate)));

    // Re gyro
    controller.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Aiming Controls
    controller.povUp().onTrue(drivetrain.selectPiece("Coral"));
    controller.povDown().onTrue(drivetrain.selectPiece("Algae"));

    controller.povLeft().onTrue(drivetrain.selectReef("Left"));
    controller.povRight().onTrue(drivetrain.selectReef("Right"));

    // Drive to pose
    controller.rightBumper().whileTrue(autoAim());
   

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void autonomousInit() {
    autoChooser.getSelected().cmd().schedule();
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    drivetrain.getNearestReef();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    poseEstimation();

    // Logging
    DogLog.log("Vision/Nearest Reef", drivetrain.getNearestReef());

    // LED comms for selected peice 
    if (drivetrain.getSelectedPiece().equals("Coral")) {
      if (drivetrain.getSelectedReef().equals("Left")) {
        leds.setPattern(leds.leftCoral);
      } else if (drivetrain.getSelectedReef().equals("Right")) {
        leds.setPattern(leds.rightCoral);
      } else {
        // No reef selected
        leds.setPattern(leds.white);
      }
    } else if (drivetrain.getSelectedPiece().equals("Algae")) {
      leds.setPattern(leds.cyan);
    } else {
      leds.setPattern(leds.black);
    }
  }
    
  @Override
  public void teleopPeriodic() {}

  public Command autoAim() {
    return Commands.sequence(
        drivetrain.resetAutoAimPID(), 
        drivetrain.goToPose(
          () -> drivetrain.getNearestReef()));
  }

  public void poseEstimation() {
    var visionLeftEst = visionLeft.getEstimatedGlobalPose(visionLeft.getCamera());
    var visionRightEst = visionRight.getEstimatedGlobalPose(visionRight.getCamera());

    visionLeftEst.ifPresent(
        est -> {
          var estStdDevs = visionLeft.getEstimationStdDevs();
          DogLog.log("Vision/LeftCam Estimated Pose", est.estimatedPose);
          drivetrain.addVisionMeasurement(
              est.estimatedPose.toPose2d(),
              Utils.fpgaToCurrentTime(est.timestampSeconds),
              estStdDevs);
        });

    visionRightEst.ifPresent(
        est -> {
          var estStdDevs = visionRight.getEstimationStdDevs();
          DogLog.log("Vision/RightCam Estimated Pose", est.estimatedPose);
          drivetrain.addVisionMeasurement(
              est.estimatedPose.toPose2d(),
              Utils.fpgaToCurrentTime(est.timestampSeconds),
              estStdDevs);
        });
  }

  // Auto paths
  public AutoRoutine midl4() {

    AutoRoutine routine = autoFactory.newRoutine("Mid l4");
    AutoTrajectory trajectory = routine.trajectory("Mid l4");

    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
    return routine;
  }

  public AutoRoutine doublel4() {

    AutoRoutine routine = autoFactory.newRoutine("Double l4");
    AutoTrajectory trajectory = routine.trajectory("Double l4");

    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
    return routine;
  }
}
