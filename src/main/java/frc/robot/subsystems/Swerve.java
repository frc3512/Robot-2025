package frc.robot.subsystems;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.DriveConstants.TunerSwerveDrivetrain;
import java.util.function.Supplier;

@SuppressWarnings("unused")
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  private boolean m_hasAppliedOperatorPerspective = false;

  // Choreo
  private final PIDController choreoXController =
      new PIDController(
          Constants.AutoConstants.xP, Constants.AutoConstants.xI, Constants.AutoConstants.xD);
  private final PIDController choreoYController =
      new PIDController(
          Constants.AutoConstants.yP, Constants.AutoConstants.xI, Constants.AutoConstants.xD);
  private final PIDController choreoThetaController =
      new PIDController(
          Constants.AutoConstants.thetaP,
          Constants.AutoConstants.thetaI,
          Constants.AutoConstants.thetaD);

  // Aiming
  private final ProfiledPIDController aimXController =
      new ProfiledPIDController(
          Constants.AimingConstants.xP,
          Constants.AimingConstants.xI,
          Constants.AimingConstants.xD,
          Constants.AimingConstants.aimingTranslationConstraints);
  private final ProfiledPIDController aimYController =
      new ProfiledPIDController(
          Constants.AimingConstants.yP,
          Constants.AimingConstants.xI,
          Constants.AimingConstants.xD,
          Constants.AimingConstants.aimingTranslationConstraints);
  private final ProfiledPIDController aimThetaController =
      new ProfiledPIDController(
          Constants.AimingConstants.thetaP,
          Constants.AimingConstants.thetaI,
          Constants.AimingConstants.thetaD,
          Constants.AimingConstants.aimingRotationConstraints);

  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    choreoThetaController.enableContinuousInput(-Math.PI, Math.PI);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    aimXController.setTolerance(Units.inchesToMeters(0.5), Units.inchesToMeters(0.125));
    aimYController.setTolerance(Units.inchesToMeters(0.5), Units.inchesToMeters(0.125));
    aimThetaController.setTolerance(Units.degreesToRadians(2.0), Units.degreesToRadians(1));

    choreoThetaController.enableContinuousInput(-Math.PI, Math.PI);
    aimThetaController.enableContinuousInput(-Math.PI, Math.PI);
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
    aimXController.setTolerance(Units.inchesToMeters(0.5));
    aimYController.setTolerance(Units.inchesToMeters(0.5));
    aimThetaController.setTolerance(Units.degreesToRadians(2.0));

    choreoThetaController.enableContinuousInput(-Math.PI, Math.PI);
    aimThetaController.enableContinuousInput(-Math.PI, Math.PI);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public void followTrajectory(SwerveSample sample) {
    Pose2d pose = getState().Pose;

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + choreoXController.calculate(pose.getX(), sample.x),
            sample.vy + choreoYController.calculate(pose.getY(), sample.y),
            sample.omega
                + choreoThetaController.calculate(pose.getRotation().getRadians(), sample.heading));

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

  public Command resetAutoAimPID() {
    return Commands.runOnce(
        () -> {
          aimXController.reset(getState().Pose.getX());
          aimYController.reset(getState().Pose.getY());
          aimThetaController.reset(getState().Pose.getRotation().getRadians());
        });
  }

  public void controlPosition(Pose2d targetPose) {
    double x = aimXController.calculate(getState().Pose.getX(), targetPose.getX());
    double y = aimYController.calculate(getState().Pose.getY(), targetPose.getY());
    double rot =
        aimThetaController.calculate(
            getState().Pose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getState().Pose.getRotation());
        
    this.setControl(
        new SwerveRequest.FieldCentric()
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  public Command goToPose(Supplier<Pose2d> target) {
    return this.run(() -> controlPosition(target.get()))
        .until(() -> false)
        .andThen(Commands.runOnce(() -> this.applyRequest(new SwerveRequest.RobotCentric())));
  }

  @Override
  public void periodic() {
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
