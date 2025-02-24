package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
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
import frc.robot.subsystems.LEDs;
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

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  // Subsystem Objects
  public final Climber climber = new Climber();
  public final Elevator elevator = new Elevator();
  public final Groundtake groundtake = new Groundtake();
  public final LEDs leds = new LEDs();
  public final Reeftake reeftake = new Reeftake();
  public final Swerve drivetrain = DriveConstants.createDrivetrain();
  // public final Vision vision = new Vision();

  // Controller Objects
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick appendageJoystick = new CommandJoystick(1);

  // Auton
  private final AutoFactory autoFactory;

  public Robot() {

    // Create Choreo
    autoFactory =
        new AutoFactory(
            () -> drivetrain.getState().Pose,
            drivetrain::resetPose,
            drivetrain::followTrajectory,
            true,
            drivetrain);

    // autoChooser.addOption("Forward", testAuto());
    autoChooser.addOption("Orbit Reef", orbitReef());

    // Controler Bindings
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-controller.getLeftY() * MaxSpeed)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed)
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate)));

    controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
    controller.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    controller.b().onTrue(leds.runPattern(leds.blue));
    controller.y().onTrue(leds.runPattern(leds.scrollngRainbow));

    // // Bindings for the button box

    // Intake control
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
    
    // Reeftake Stuff
    appendageJoystick.button(10).onTrue(new InstantCommand(() -> reeftake.coralIntake()));
    appendageJoystick.button(10).onFalse(new InstantCommand(() -> reeftake.coralStop()));

    // // Elevator controls
    appendageJoystick.button(6).onTrue(new InstantCommand(() -> elevator.l1()));

    appendageJoystick.button(5).onTrue(new InstantCommand(() -> elevator.l2()));

    appendageJoystick.button(4).onTrue(new InstantCommand(() -> elevator.l3()));

    appendageJoystick.button(3).onTrue(new InstantCommand(() -> elevator.l4()));

    appendageJoystick.button(8).onTrue(new InstantCommand(() -> elevator.a1()));

    appendageJoystick.button(7).onTrue(new InstantCommand(() -> elevator.a2()));

    appendageJoystick.button(9).onTrue(new InstantCommand(() -> elevator.stow()));

    appendageJoystick.button(12).onTrue(new InstantCommand(() -> elevator.hp()));

    // Manual control, COMMENT OUT WHEN NOT IN USE
    // appendageJoystick.button(7).onTrue(new InstantCommand(() -> elevator.elevatorUp()));
    // appendageJoystick.button(7).onFalse(new InstantCommand(() -> elevator.elevatorStop()));

    // appendageJoystick.button(8).onTrue(new InstantCommand(() -> elevator.elevatorDown()));
    // appendageJoystick.button(8).onFalse(new InstantCommand(() -> elevator.elevatorStop()));


    // Climber controls
    appendageJoystick.button(1).onTrue(climber.setClimber(0.8));
    appendageJoystick.button(1).onFalse(climber.setClimber(0.0));

    appendageJoystick.button(2).onTrue(climber.setClimber(-0.8));
    appendageJoystick.button(2).onFalse(climber.setClimber(0.0));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void autonomousInit() {
    autoChooser.getSelected().cmd().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {}

  // public AutoRoutine testAuto() {

  //   AutoRoutine routine = autoFactory.newRoutine("Forward");
  //   AutoTrajectory testTrajectory = routine.trajectory("Forward");

  //   routine
  //       .active()
  //       .onTrue(Commands.sequence(testTrajectory.resetOdometry(), testTrajectory.cmd()));

  //   return routine;
  // }

  public AutoRoutine orbitReef() {

    AutoRoutine routine = autoFactory.newRoutine("Orbit Reef");
    AutoTrajectory testTrajectory = routine.trajectory("Orbit Reef");

    routine
        .active()
        .onTrue(Commands.sequence(testTrajectory.resetOdometry(), testTrajectory.cmd()));

    return routine;
  }
}
