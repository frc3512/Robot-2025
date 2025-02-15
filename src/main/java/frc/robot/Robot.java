package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

public class Robot extends TimedRobot {

  private double MaxSpeed = DriveConstants.MaxSpeed;
  private double MaxAngularRate = DriveConstants.MaxAngularRate;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.07) // Add a 7% deadband
          .withDriveRequestType(DriveRequestType.Velocity);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Subsystem Objects
  public final Climber climber = new Climber();
  public final Elevator elevator = new Elevator();
  public final Reeftake reeftake = new Reeftake();
  public final Groundtake groundtake = new Groundtake();
  public final Swerve drivetrain = DriveConstants.createDrivetrain();
  public final Vision vision = new Vision();

  // Controller Objects
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick appendageJoystick = new CommandJoystick(1);

  // PID Controllers ( auton )
  PIDController xController = new PIDController(10, 0.0, 0.0);
  PIDController yController = new PIDController(10, 0.0, 0.0);
  PIDController headingController = new PIDController(7.5, 0.0, 0.0);

  // Auton
  private final AutoFactory autoFactory;

  public Robot() {

    // Create Choreo 
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    autoFactory =
        new AutoFactory(
            () -> drivetrain.getState().Pose,
            drivetrain::resetPose,
            drivetrain::followTrajectory,
            true,
            drivetrain);

  // Controler Bindings 
  drivetrain.setDefaultCommand(
    drivetrain.applyRequest(
        () ->
            drive
                .withVelocityX(-controller.getLeftY() * MaxSpeed)
                .withVelocityY(-controller.getLeftX() * MaxSpeed)
                .withRotationalRate(-controller.getRightX() * MaxAngularRate)));

    
        // Bindings for the controller
    controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
    controller.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Bindings for the button box

    // Intake control
    controller
        .leftTrigger()
        .onTrue(
            new InstantCommand(() -> groundtake.extendPivot())
                .andThen(new InstantCommand(() -> groundtake.floorAlgaeIntake())));
    controller.leftTrigger().onFalse(new InstantCommand(() -> groundtake.retractPivot()));

    controller.rightTrigger().onTrue(new InstantCommand(() -> groundtake.floorAlgaeOuttake()));
    controller.rightTrigger().onFalse(new InstantCommand(() -> groundtake.floorAlgaeStop()));

    drivetrain.registerTelemetry(logger::telemeterize);

    // Elevator controls
    appendageJoystick.button(6).onTrue(new InstantCommand(() -> elevator.l1()));

    appendageJoystick.button(5).onTrue(new InstantCommand(() -> elevator.l2()));

    appendageJoystick.button(4).onTrue(new InstantCommand(() -> elevator.l3()));

    appendageJoystick.button(3).onTrue(new InstantCommand(() -> elevator.l4()));

    appendageJoystick.button(8).onTrue(new InstantCommand(() -> elevator.a1()));

    appendageJoystick.button(7).onTrue(new InstantCommand(() -> elevator.a2()));

  }

  @Override
  public void autonomousInit() {
    testAuto().cmd().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {}

  public AutoRoutine testAuto() {

    AutoRoutine testRoutine = autoFactory.newRoutine("Forward");
    AutoTrajectory testTrajectory = testRoutine.trajectory("Forward");

    testRoutine
      .active()
      .onTrue(Commands.sequence(testTrajectory.resetOdometry(), testTrajectory.cmd()));

    return testRoutine;
  }

}
