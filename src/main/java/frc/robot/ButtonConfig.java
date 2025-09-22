package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

public class ButtonConfig {

  static CommandXboxController controller = new CommandXboxController(0);
  static CommandJoystick appendageJoystick = new CommandJoystick(1);

  private double maxSpeed = DriveConstants.maxSpeed;
  private double maxAngularRate = DriveConstants.maxAngularRate;
  private double slowSpeed = DriveConstants.slowSpeed;
  private double slowAngularRate = DriveConstants.slowAngularRate;

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

  public final Superstructure actions = new Superstructure();
  public final Swerve drivetrain = DriveConstants.createDrivetrain();

  public void telopInit() {

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
  }
}
