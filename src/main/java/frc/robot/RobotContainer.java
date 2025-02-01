package frc.robot;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Vision;

@SuppressWarnings("unused")
public class RobotContainer {
    
  private double MaxSpeed = DriveConstants.MaxSpeed; 
  private double MaxAngularRate = DriveConstants.MaxAngularRate;
  
  public boolean isRed() {

    if (DriverStation.getAlliance().isPresent()) {

      return DriverStation.getAlliance().get() ==  Alliance.Red;

    }

    return false;

    }
  
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.07) // Add a 7% deadband
    .withDriveRequestType(DriveRequestType.Velocity); 

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Subsystem Objects
  public final Climber climber = new Climber();
  public final Elevator elevator = new Elevator();
  public final Intake intake = new Intake();
  public final Swerve drivetrain = DriveConstants.createDrivetrain();
  // public final Vision vision = new Vision();

  // Controller Objects
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick appendageJoystick = new CommandJoystick(1);

  // PID Controllers ( auton )
  PIDController xPID = new PIDController(3, 0, 0);
  PIDController yPID = new PIDController(3, 0, 0);
  PIDController rPID = new PIDController(3, 0, 0);


  public RobotContainer() {

    configureBindings();
    configureAxisActions();

  }

  private void configureBindings() {

    // Bindings for the controller
    controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
    controller.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Vison alignment drive command
    // controller.leftBumper().whileTrue(drivetrain.applyRequest(() -> 
      // drive.withVelocityX(DriveConstants.forward)
        // .withVelocityY(DriveConstants.strafe)
        // .withRotationalRate((vision.getYawOffset() - vision.getTargetYaw()) * Constants.VisionConstants.visionTurnP * MaxAngularRate)));
    
    // Bindings for the appendage joystick

    // Elevator control
    appendageJoystick.button(1).onTrue(new InstantCommand(() -> elevator.elevatorUp()));
    appendageJoystick.button(1).onFalse(new InstantCommand(() -> elevator.elevatorStop()));

    appendageJoystick.button(4).onTrue(new InstantCommand(() -> elevator.elevatorDown()));
    appendageJoystick.button(4).onFalse(new InstantCommand(() -> elevator.elevatorStop()));

    // appendageJoystick.button(2).onTrue(new InstantCommand(() -> elevator.l1()));

    // Climber control
    appendageJoystick.button(10).onTrue(new InstantCommand(() -> climber.climbUp()));
    appendageJoystick.button(10).onFalse(new InstantCommand(() -> climber.climbStop()));

    appendageJoystick.button(11).onTrue(new InstantCommand(() -> climber.climbDown()));
    appendageJoystick.button(11).onFalse(new InstantCommand(() -> climber.climbStop()));

    // Intake control
    appendageJoystick.button(2).onTrue(new InstantCommand(() -> intake.floorAlgaeIntake()));
    appendageJoystick.button(2).onFalse(new InstantCommand(() -> intake.floorAlgaeStop()));

    appendageJoystick.button(3).onTrue(new InstantCommand(() -> intake.floorAlgaeOuttake()));
    appendageJoystick.button(3).onFalse(new InstantCommand(() -> intake.floorAlgaeStop()));
    
    appendageJoystick.button(5).onTrue(new InstantCommand(() -> intake.reefAlgaeIntake()));
    appendageJoystick.button(5).onFalse(new InstantCommand(() -> intake.reefAlgaeStop()));

    appendageJoystick.button(6).onTrue(new InstantCommand(() -> intake.reefAlgaeOuttake()));
    appendageJoystick.button(6).onFalse(new InstantCommand(() -> intake.reefAlgaeStop()));    

    drivetrain.registerTelemetry(logger::telemeterize);

  }
  
  private void configureAxisActions() {

     drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() -> drive.withVelocityX(-controller.getLeftY() * MaxSpeed)
            .withVelocityY(-controller.getLeftX() * MaxSpeed)
            .withRotationalRate(-controller.getRightX() * MaxAngularRate)));

  }

  public Command getAutonomousCommand() {

    return null;

  }

}