package frc.robot.subsystems;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.DriveConstants.TunerSwerveDrivetrain;

import java.lang.reflect.Field;
import java.util.function.Supplier;

@SuppressWarnings("unused")
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
  private final Field2d robotView = new Field2d();

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  private boolean m_hasAppliedOperatorPerspective = false;

  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {

    super(drivetrainConstants, modules);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putData("Position", robotView);

    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {

    super(drivetrainConstants, odometryUpdateFrequency, modules);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    SmartDashboard.putData("Position", robotView);

    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {

    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putData("Position", robotView);

    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  // Choreo Stuff
  private final PIDController xController = new PIDController(10, 0.0, 0.0);
  private final PIDController yController = new PIDController(10, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(7.5, 0.0, 0.0);

  public void followTrajectory(SwerveSample sample) {

    Pose2d pose = getState().Pose;

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega
                + thetaController.calculate(pose.getRotation().getRadians(), sample.heading));

    this.setControl(
        new SwerveRequest.FieldCentric()
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public void applyRequest(SwerveRequest request) {
    this.setControl(request);
  }

  @Override
  public void periodic() {

    // if ((!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) &&
    // DriverStation.isTeleopEnabled()) {
    //   DriverStation.getAlliance()
    //       .ifPresent(
    //           allianceColor -> {
    //             setOperatorPerspectiveForward(
    //                 allianceColor == Alliance.Red
    //                     ? kRedAlliancePerspectiveRotation
    //                     : kBlueAlliancePerspectiveRotation);
    //             m_hasAppliedOperatorPerspective = true;
    //           });

    // if(DriverStation.isAutonomousEnabled()){
    //   DriverStation.getAlliance()
    //   .ifPresent(
    //     allianceColor -> {

    //       setOperatorPerspectiveForward(kBlueAlliancePerspectiveRotation);
    //         m_hasAppliedOperatorPerspective=false;
    //     });
    // }

    // }

    // Log General Swerve Information
    DogLog.log("Swerve/ModuleStates", getState().ModuleStates);
    DogLog.log("Swerve/ModuleStateSetpoints", getState().ModuleTargets);
    DogLog.log("Swerve/OdometryPose", getState().Pose);
    DogLog.log("Swerve/ChassisSpeeds", getState().Speeds);
    // Module Name Keys
    String[] moduleNames = new String[] {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
    // Log Module Data
    for (int i = 0; i < 4; i++) {
      DogLog.log(
          "Swerve/Modules/" + moduleNames[i] + "/EncoderAbsolutePosition",
          getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble());
      DogLog.log(
          "Swerve/Modules/" + moduleNames[i] + "/EncoderPosition",
          getModule(i).getEncoder().getPosition().getValueAsDouble());
      DogLog.log(
          "Swerve/Modules/" + moduleNames[i] + "/DriveVelocity",
          getModule(i).getCurrentState().speedMetersPerSecond);
      DogLog.log(
          "Swerve/Modules/" + moduleNames[i] + "/DriveVelocitySetpoint",
          getModule(i).getTargetState().speedMetersPerSecond);
      DogLog.log(
          "Swerve/Modules/" + moduleNames[i] + "/DriveSupplyCurrent",
          getModule(i).getDriveMotor().getSupplyCurrent().getValueAsDouble());
      DogLog.log(
          "Swerve/Modules/" + moduleNames[i] + "/DriveStatorCurrent",
          getModule(i).getDriveMotor().getStatorCurrent().getValueAsDouble());
      DogLog.log(
          "Swerve/Modules/" + moduleNames[i] + "/DriveVoltage",
          getModule(i).getDriveMotor().get() * RobotController.getBatteryVoltage());
      DogLog.log(
          "Swerve/Modules/" + moduleNames[i] + "/DriveTemperature",
          getModule(i).getDriveMotor().getDeviceTemp().getValueAsDouble());
      DogLog.log(
          "Swerve/Modules/" + moduleNames[i] + "/TurnPosition",
          getModule(i).getCurrentState().angle.getRadians());
      DogLog.log(
          "Swerve/Modules/" + moduleNames[i] + "/TurnPositionSetpoint",
          getModule(i).getTargetState().angle.getRadians());
      DogLog.log(
          "Swerve/Modules/" + moduleNames[i] + "/TurnSupplyCurrent",
          getModule(i).getSteerMotor().getSupplyCurrent().getValueAsDouble());
      DogLog.log(
          "Swerve/Modules/" + moduleNames[i] + "/TurnStatorCurrent",
          getModule(i).getSteerMotor().getStatorCurrent().getValueAsDouble());
      DogLog.log(
          "Swerve/Modules/" + moduleNames[i] + "/TurnVoltage",
          getModule(i).getSteerMotor().get() * RobotController.getBatteryVoltage());
      DogLog.log(
          "Swerve/Modules/" + moduleNames[i] + "/TurnTemperature",
          getModule(i).getSteerMotor().getDeviceTemp().getValueAsDouble());

      robotView.setRobotPose(getState().Pose);
    }
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }
}
