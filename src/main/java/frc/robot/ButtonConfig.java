package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
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
  public final Intake intake = new Intake();

  public Command autoAim() {
    return Commands.sequence(
        drivetrain.resetAutoAimPID(), 
        drivetrain.goToPose(
          () -> drivetrain.getNearestReef()));
  }

  public void configureButtons() {

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-controller.getLeftY() * maxSpeed)
                    .withVelocityY(-controller.getLeftX() * maxSpeed)
                    .withRotationalRate(-controller.getRightX() * maxAngularRate)));

    // Slow drive button
    controller
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    driveSlow
                        .withVelocityX(-controller.getLeftY() * slowSpeed)
                        .withVelocityY(-controller.getLeftX() * slowSpeed)
                        .withRotationalRate(-controller.getRightX() * slowAngularRate)));

    controller.leftTrigger()
        .whileTrue(actions.INTAKE_CORAL()
            .until(() -> intake.hasCoral())
            .andThen(actions.PREP_CORAL()))
        .onFalse(actions.RESET());

    controller.rightTrigger()
        .whileTrue(actions.INTAKE_ALGAE()
            .until(() -> intake.hasAlgae())
            .andThen(actions.PREP_ALGAE()))
        .onFalse(actions.RESET());

    // * Manual Vision * 
    // controller.y()
    //     .onTrue(drivetrain.selectPiece("Coral"));
    // controller.a()
    //     .onTrue(drivetrain.selectPiece("Algae"));

    // controller.leftBumper()
    //     .onTrue(drivetrain.selectReef("Left"));
    // controller.rightBumper()
    //     .onTrue(drivetrain.selectReef("Right"));

    // controller.x()
    //     .onTrue(autoAim());

    // * -- Bindings for the button box --

    // * Coral
    // appendageJoystick.button(3)
    //     .onTrue(actions.SCORE(ElevatorStates.L4));

    // appendageJoystick.button(4)
    //     .onTrue(actions.SCORE(ElevatorStates.L3));
    
    // appendageJoystick.button(5)
    //     .onTrue(actions.SCORE(ElevatorStates.L2));

    // * Algae
    // appendageJoystick.button(8)
    //     .onTrue(actions.GRAB_ALGAE_L1());
    
    // appendageJoystick.button(7)
    //     .onTrue(actions.GRAB_ALGAE_L2());

    // appendageJoystick.button(10)
    //     .onTrue(actions.PREP_BARGE())
    //      .onFalse(actions.SCORE_BARGE());

    // Reset
    // Function in case drivers press wrong button and need a kill switch
    // appendageJoystick.button(12)
    //     .onTrue(actions.RESET());

    // ! TESTING PURPOSES ONLY
    // ! COMMENT OUT WHEN NOT TESTING

    // * Elevator * 
    // appendageJoystick.button(3)
    //     .onTrue(actions.L4());
    
    // appendageJoystick.button(4)
    //     .onTrue(actions.L3());
    
    // appendageJoystick.button(5)
    //     .onTrue(actions.L2());
    
    // appendageJoystick.button(6)
    //     .onTrue(actions.L1());

    // appendageJoystick.button(8)
    //     .onTrue(actions.ALGAE_L1());
    
    // appendageJoystick.button(7)
    //     .onTrue(actions.ALGAE_L2());
    
    // * Arm *

    // * Wrist *
    // appendageJoystick.button(1)
    //     .onTrue(actions.WRIST_VERTICAL());

    // appendageJoystick.button(2)
    //     .onTrue(actions.WRIST_HORIZONTAL());

  }
}
