package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.Swerve;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private SendableChooser<AutoRoutine> autoChooser = new SendableChooser<>();

  // Drivetrain
  private final Swerve drivetrain = DriveConstants.createDrivetrain();

  // Auton
  private final AutoFactory autoFactory;

  // Subsystems
  private Elevator elevator;

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

        break;

      case SIM:
        Elevator.setInstance(new ElevatorIOSim());
        elevator = Elevator.getInstance();

        break;

      case REPLAY:
        Elevator.setInstance(new ElevatorIO() {});
        elevator = Elevator.getInstance();

        break;
    }

    // Create Choreo
    autoFactory =
        new AutoFactory(
            () -> drivetrain.getState().Pose,
            drivetrain::resetPose,
            drivetrain::followTrajectory,
            false,
            drivetrain);

    autoChooser.addOption("Mid l4", midl4());

    SmartDashboard.putData("Auto Chooser", autoChooser);

    Logger.start();
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
