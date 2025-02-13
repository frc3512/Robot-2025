package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.Telemetry;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Groundtake;
import frc.robot.subsystems.Reeftake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

@SuppressWarnings("unused")
public class RobotContainer<DriveSubsystem> {

  private double MaxSpeed = DriveConstants.MaxSpeed;
  private double MaxAngularRate = DriveConstants.MaxAngularRate;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.07) // Add a 7% deadband
          .withDriveRequestType(DriveRequestType.Velocity);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Subsystem Objects
  public final Climber climber = new Climber();
  public final Elevator elevator = new Elevator();
  public final Reeftake reektake = new Reeftake();
  public final Groundtake groundtake = new Groundtake();
  public final Swerve drivetrain = DriveConstants.createDrivetrain();
  public final Vision vision = new Vision();

  // Controller Objects
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick appendageJoystick = new CommandJoystick(1);

  // PID Controllers ( auton )
  PIDController xController = new PIDController(1, 0.0, 0.0);
  PIDController yController = new PIDController(1, 0.0, 0.0);
  PIDController headingController = new PIDController(0.75, 0.0, 0.0);

  // Auton
  private final AutoFactory autoFactory;

  public RobotContainer() {
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    autoFactory =
        new AutoFactory(
            () -> drivetrain.getState().Pose,
            drivetrain::resetPose,
            this::followTrajectory,
            true,
            drivetrain);

    configureBindings();
    configureAxisActions();
    configureVision();

    Command Test = autoFactory.trajectoryCmd("Forward");
    Command Trajectory1 = autoFactory.trajectoryCmd("Score 3pt go H");
    Command Trajectory2 = autoFactory.trajectoryCmd("Score pt go H");

  }

  private void configureBindings() {

    // Bindings for the controller
    controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
    controller.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Bindings for the button box
    // Elevator control
    appendageJoystick.button(1).onTrue(new InstantCommand(() -> elevator.elevatorUp()));
    appendageJoystick.button(1).onFalse(new InstantCommand(() -> elevator.elevatorStop()));

    appendageJoystick.button(4).onTrue(new InstantCommand(() -> elevator.elevatorDown()));
    appendageJoystick.button(4).onFalse(new InstantCommand(() -> elevator.elevatorStop()));

    // Climber control
    appendageJoystick.button(10).onTrue(new InstantCommand(() -> climber.climbUp()));
    appendageJoystick.button(10).onFalse(new InstantCommand(() -> climber.climbStop()));

    appendageJoystick.button(11).onTrue(new InstantCommand(() -> climber.climbDown()));
    appendageJoystick.button(11).onFalse(new InstantCommand(() -> climber.climbStop()));

    // Intake control
    controller.leftTrigger().onTrue(new InstantCommand(() -> groundtake.floorAlgaeIntake()));
    controller.leftTrigger().onFalse(new InstantCommand(() -> groundtake.floorAlgaeStop()));

    controller.rightTrigger().onTrue(new InstantCommand(() -> groundtake.floorAlgaeOuttake()));
    controller.rightTrigger().onFalse(new InstantCommand(() -> groundtake.floorAlgaeStop()));

    controller.y().onTrue(new InstantCommand(() -> groundtake.retractPivot()));
    controller.b().onTrue(new InstantCommand(() -> groundtake.extendPivot()));

    appendageJoystick.button(5).onTrue(new InstantCommand(() -> reektake.reefAlgaeIntake()));
    appendageJoystick.button(5).onFalse(new InstantCommand(() -> reektake.reefAlgaeStop()));

    appendageJoystick.button(6).onTrue(new InstantCommand(() -> reektake.reefAlgaeOuttake()));
    appendageJoystick.button(6).onFalse(new InstantCommand(() -> reektake.reefAlgaeStop()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureAxisActions() {

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-controller.getLeftY() * MaxSpeed)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed)
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate)));
  }

  private void configureVision() {

    // Vison alignment drive command
    controller.leftBumper().whileTrue(
      drivetrain.applyRequest(
        () ->
            drive
                .withVelocityX(DriveConstants.forward)
                .withVelocityY(DriveConstants.strafe)
                .withRotationalRate((vision.getYawOffset() - vision.getTargetYaw()) * 
                Constants.VisionConstants.visionTurnP * MaxAngularRate)));

  }

  public void followTrajectory(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d pose = drivetrain.getState().Pose;

    // Generate the next speeds for the robot
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega
                + headingController.calculate(pose.getRotation().getRadians(), sample.heading));

    // Apply the generated speeds
    drivetrain.applyRequest(
        () ->
            drive
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  public Command getAutonomousCommand() {
    Command Test = autoFactory.trajectoryCmd("Test");
    return Test;
  }

  public AutoRoutine testAuto(AutoFactory autoFactory2) {
    final AutoRoutine routine = autoFactory2.newRoutine("Forward");

    final AutoTrajectory trajectory = routine.trajectory("Forward");
    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));

    return routine;
  }
}
