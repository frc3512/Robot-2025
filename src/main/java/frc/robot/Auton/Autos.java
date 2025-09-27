package frc.robot.Auton;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Autos {

  private final Swerve swerve;
  private final SendableChooser<Command> autonChooser;

  public Autos(Swerve swerve) {
    this.swerve = swerve;

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
        swerve::getPose,                // Pose supplier
        swerve::resetOdometry,          // Reset pose
        swerve::getRobotRelativeSpeeds, // Robot-relative chassis speeds
        this::driveRobotRelative,       // Drive method
        new PPHolonomicDriveController(
            new PIDConstants(Constants.AutoConstants.xP, Constants.AutoConstants.xI, Constants.AutoConstants.xD), // Translation PID
            new PIDConstants(Constants.AutoConstants.thetaP, Constants.AutoConstants.thetaI, Constants.AutoConstants.thetaD)  // Rotation PID
        ),
        config,
        () -> DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
        swerve // requirement
    );

    autonChooser = new SendableChooser<>();
    autonChooser.setDefaultOption("No-op", new InstantCommand());

    // Example auto
    buildAuto("Test Auto");

    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  public Command getSelected() {
    return autonChooser.getSelected();
  }

  private void buildAuto(String autoName) {
    Command autoCommand = AutoBuilder.buildAuto(autoName);
    autonChooser.addOption(autoName, autoCommand);
  }
    
  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();
    request.Speeds = speeds;
    swerve.setControl(request);
  }
}
