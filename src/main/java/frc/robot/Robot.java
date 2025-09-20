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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

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
  public final Superstructure actions = new Superstructure();
  public final Swerve drivetrain = DriveConstants.createDrivetrain();

  public final LED leds;
  public final Elevator elevator;

  // Commands

  // Controller Objects
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick appendageJoystick = new CommandJoystick(1);

  // Auton
  private final AutoFactory autoFactory;

  public Robot() {

    Elevator.setInstance(
        Constants.ElevatorConstants.leadID, Constants.ElevatorConstants.followerID);
    elevator = Elevator.getInstance();

    leds = new LED();

    // Create Choreo
    autoFactory =
        new AutoFactory(
            () -> drivetrain.getState().Pose,
            drivetrain::resetPose,
            drivetrain::followTrajectory,
            false,
            drivetrain);
    autoChooser.addOption("Mid l4", midl4());

    //  * -- Controller Bindings --

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-controller.getLeftY() * maxSpeed)
                    .withVelocityY(-controller.getLeftX() * maxSpeed)
                    .withRotationalRate(-controller.getRightX() * maxAngularRate)));

    // Slow drive button
    controller
        .rightBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    driveSlow
                        .withVelocityX(-controller.getLeftY() * slowSpeed)
                        .withVelocityY(-controller.getLeftX() * slowSpeed)
                        .withRotationalRate(-controller.getRightX() * slowAngularRate)));

    // * -- Bindings for the button box --

    // | Elevator Control |

    // Coral
    appendageJoystick.button(3).onTrue(actions.L4());

    appendageJoystick.button(4).onTrue(actions.L3());

    appendageJoystick.button(5).onTrue(actions.L2());

    appendageJoystick.button(6).onTrue(actions.L1());

    // Algae
    appendageJoystick.button(7).onTrue(actions.ALGAE_L1());

    appendageJoystick.button(8).onTrue(actions.ALGAE_L2());

    appendageJoystick.button(10).onTrue(actions.BARGE());

    // appendageJoystick.button(11)
    //     .onTrue(actions.SPIT_ALGAE());

    // Defaults
    appendageJoystick.button(9).onTrue(actions.STOW());

    // appendageJoystick.button(12)
    //     .onTrue(actions.INTAKE_CORAL());

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void autonomousInit() {
    autoChooser.getSelected().cmd().schedule();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {}

  // Auto paths
  public AutoRoutine midl4() {

    AutoRoutine routine = autoFactory.newRoutine("Mid l4");
    AutoTrajectory trajectory = routine.trajectory("Mid l4");

    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
    return routine;
  }
}
