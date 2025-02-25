package frc.robot;

import choreo.auto.AutoChooser;
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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Groundtake;
import frc.robot.subsystems.LEDs;
// import frc.robot.subsystems.Reeftake;
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
  // public final Climber climber = new Climber();
  // public final Elevator elevator = new Elevator();
  // public final Groundtake groundtake = new Groundtake();
  public final LEDs leds = new LEDs();
  // public final Reeftake reeftake = new Reeftake();
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

    autoChooser.addOption("Forward(Center)", forward1());
    autoChooser.addOption("Forward(Top)", forward2());
    autoChooser.addOption("Forward(Bottom)", forward3());
    autoChooser.addOption("Orbit Reef(Test)", orbitReef());
    autoChooser.addOption("Hp(Bottom)", hPB());
    autoChooser.addOption("Hp(Top)", hPT());
    // autoChooser.addOption("Score l4 3x", l4Threex());
    autoChooser.addOption("Score L4 go S AP", L4GoSAP());
    autoChooser.addOption("Score l4 go HP Score l4 Finish S", L4HP2xFinS());

    // Controler Bindings
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-controller.getLeftY() * MaxSpeed)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed)
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate)));

    // Bindings for the controller
    controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
    controller.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    controller.y().onTrue(new InstantCommand(() -> leds.blue()));
    controller.b().onTrue(new InstantCommand(() -> leds.rainbow()));

    // // Bindings for the button box

    // // Intake control
    // controller
    //     .leftTrigger()
    //     .onTrue(
    //         new InstantCommand(() -> groundtake.extendPivot())
    //             .andThen(new InstantCommand(() -> groundtake.floorAlgaeIntake())));
    // controller.leftTrigger().onFalse(new InstantCommand(() -> groundtake.retractPivot()));

    // controller.rightTrigger().onTrue(new InstantCommand(() -> groundtake.floorAlgaeOuttake()));
    // controller.rightTrigger().onFalse(new InstantCommand(() -> groundtake.floorAlgaeStop()));

    // // Elevator controls
    // appendageJoystick.button(6).onTrue(new InstantCommand(() -> elevator.l1()));

    // appendageJoystick.button(5).onTrue(new InstantCommand(() -> elevator.l2()));

    // appendageJoystick.button(4).onTrue(new InstantCommand(() -> elevator.l3()));

    // appendageJoystick.button(3).onTrue(new InstantCommand(() -> elevator.l4()));

    // appendageJoystick.button(8).onTrue(new InstantCommand(() -> elevator.a1()));

    // appendageJoystick.button(7).onTrue(new InstantCommand(() -> elevator.a2()));

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

  public AutoRoutine forward1() {

    AutoRoutine routine = autoFactory.newRoutine("Forward(Center)");
    AutoTrajectory trajectory = routine.trajectory("Forward(Center)");
    routine
      .active()
      .onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
    return routine;
  }

  public AutoRoutine forward2() {
    AutoRoutine routine = autoFactory.newRoutine("Forward(Top)");
    AutoTrajectory trajectory = routine.trajectory("Foward(Top)");
    routine
      .active()
      .onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
    return routine;
  }

  public AutoRoutine forward3() {
    AutoRoutine routine = autoFactory.newRoutine("Forward(Bottom)");
    AutoTrajectory trajectory = routine.trajectory("Foward(Bottom)");
    routine
      .active()
      .onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
    return routine;
  }

  public AutoRoutine hPB() {
    AutoRoutine routine = autoFactory.newRoutine("Hp(Bottom)");
    AutoTrajectory trajectory = routine.trajectory("Hp(Bottom)");
    routine
      .active()
      .onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
    return routine;
  }

  public AutoRoutine hPT() {
    AutoRoutine routine = autoFactory.newRoutine("Hp(Top)");
    AutoTrajectory trajectory = routine.trajectory("Hp(Top)");
    routine
      .active()
      .onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
    return routine;
  }

  public AutoRoutine orbitReef() {

    AutoRoutine routine = autoFactory.newRoutine("Orbit Reef(Test)");
    AutoTrajectory trajectory = routine.trajectory("Orbit Reef(Test)");
    routine
      .active()
      .onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));

    return routine;
  }

  // public AutoRoutine l4Threex() {
  //   AutoRoutine routine = autoFactory.newRoutine("Score L4 3x");
  //   AutoTrajectory trajectory = routine.trajectory("Score L4 3x");
  //   routine
  //     .active()
  //     .onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
  //     return routine;
  // }

  public AutoRoutine L4GoSAP() {
    AutoRoutine routine = autoFactory.newRoutine("Score L4 go S AP");
    AutoTrajectory trajectory = routine.trajectory("Score L4 go S AP");
    routine
      .active()
      .onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
    return routine;
  }

  public AutoRoutine L4HP2xFinS() {
    AutoRoutine routine = autoFactory.newRoutine("Score l4 go HP Score l4 Finish S");
    AutoTrajectory trajectory = routine.trajectory("Score l4 go HP Score l4 Finish S");
    routine
      .active()
      .onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
    return routine;
  }

}
