package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.Utils;

import dev.doglog.DogLog;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmIO;
import frc.robot.subsystems.Arm.ArmIOSim;
import frc.robot.subsystems.Arm.ArmIOTalonFX;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristIO;
import frc.robot.subsystems.Wrist.WristIOSim;
import frc.robot.subsystems.Wrist.WristIOTalonFX;

public class Robot extends LoggedRobot {

  // Subsystems
  private Swerve swerve;

  private Elevator elevator;
  private Arm arm;
  private Wrist wrist;

  private Intake intake;
  private LED leds;

  public final Vision visionLeft =
      new Vision(
          Constants.VisionConstants.leftCam, Constants.VisionConstants.leftCameraOffset);
  public final Vision visionRight =
      new Vision(
          Constants.VisionConstants.rightCam, Constants.VisionConstants.rightCameraOffset);

  public Robot() {

    switch (Constants.GeneralConstants.currentMode) {
      case REAL:

        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());

        break;

      case SIM:

        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());

        break;

      case REPLAY:

        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));

        break;
    }

    switch (Constants.GeneralConstants.currentMode) {
      case REAL:

        Elevator.setInstance(
            new ElevatorIOTalonFX(
                Constants.ElevatorConstants.leadID, Constants.ElevatorConstants.followerID));
        elevator = Elevator.getInstance();

        Arm.setInstance(
            new ArmIOTalonFX(Constants.ArmConstants.motorID));
        arm = Arm.getInstance();

        Wrist.setInstance(
          new WristIOTalonFX(Constants.WristConstants.motorID));
        wrist = Wrist.getInstance();

        intake = new Intake();
        leds = new LED();

        swerve = DriveConstants.createDrivetrain();
        // swerve.configurePathplanner();

        break;

      case SIM:

        Elevator.setInstance(new ElevatorIOSim());
        elevator = Elevator.getInstance();

        Arm.setInstance(new ArmIOSim());
        arm = Arm.getInstance();

        Wrist.setInstance(new WristIOSim());
        wrist = Wrist.getInstance();

        intake = new Intake();
        leds = new LED();

        swerve = DriveConstants.createDrivetrain();
        // swerve.configurePathplanner();

        break;

      case REPLAY:

        Elevator.setInstance(new ElevatorIO() {});
        elevator = Elevator.getInstance();

        Arm.setInstance(new ArmIO() {});
        arm = Arm.getInstance();

        Wrist.setInstance(new WristIO() {});
        wrist = Wrist.getInstance();

        break;
    }

    Logger.start();

    ButtonConfig buttons = new ButtonConfig();
    buttons.configureButtons();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void teleopInit() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    poseEstimation();

    DogLog.log("Vision/Nearest Reef", swerve.getNearestReef());

    if (intake.hasCoral()){
      swerve.selectPiece("Coral");
      
      if (swerve.getSelectedReef() == "Left") {
        leds.setPattern(leds.left);
      } else if (swerve.getSelectedReef() == "Right") {
        leds.setPattern(leds.right);
      } else {
        leds.setPattern(leds.white);
      }
    } else if (intake.hasAlgae()){
      swerve.selectPiece("Algae");
      leds.setPattern(leds.cyan);
    } else {
      leds.setPattern(leds.black);
    }
  } 


  public void poseEstimation() {
    var visionLeftEst = visionLeft.getEstimatedGlobalPose(visionLeft.getCamera());
    var visionRightEst = visionRight.getEstimatedGlobalPose(visionRight.getCamera());

    visionLeftEst.ifPresent(
        est -> {
          var estStdDevs = visionLeft.getEstimationStdDevs();
          DogLog.log("Vision/LeftCam Estimated Pose", est.estimatedPose);
          swerve.addVisionMeasurement(
              est.estimatedPose.toPose2d(),
              Utils.fpgaToCurrentTime(est.timestampSeconds),
              estStdDevs);
        });

    visionRightEst.ifPresent(
        est -> {
          var estStdDevs = visionRight.getEstimationStdDevs();
          DogLog.log("Vision/RightCam Estimated Pose", est.estimatedPose);
          swerve.addVisionMeasurement(
              est.estimatedPose.toPose2d(),
              Utils.fpgaToCurrentTime(est.timestampSeconds),
              estStdDevs);
        });
  }


  @Override
  public void teleopPeriodic() {}
}
