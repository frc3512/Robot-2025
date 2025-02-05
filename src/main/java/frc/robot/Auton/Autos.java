// package frc.robot.Auton;

// import choreo.auto.AutoFactory;
// import choreo.auto.AutoRoutine;
// import choreo.auto.AutoTrajectory;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.subsystems.Swerve;

// public class Autos extends TimedRobot{

//         private final Swerve driveSubsystem = new Swerve(null, null);
        
//         private AutoFactory autoFactory;
    
//         @Override
//         public void robotInit() {
//             autoFactory = new AutoFactory(
//                 driveSubsystem::odometry.getPoseMeters(), // A function that returns the current robot pose
//                 driveSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
//                 driveSubsystem::followTrajectory, // The drive subsystem trajectory follower 
//                 true, // If alliance flipping should be enabled 
//                 driveSubsystem // The drive subsystem
//             );
//         }

// public AutoRoutine DriveForward() {
//     // Create the routine
//     AutoRoutine routine = autoFactory.newRoutine("Test");

//     // Load the routine's trajectories
//     AutoTrajectory driveForward = routine.trajectory("test");

//     // When the routine begins, reset odometry and start the first trajectory (1)
//     routine.active().onTrue(
//         Commands.sequence(
//             driveSubsystem.resetOdometry(),
//             driveSubsystem.followTrajectory()
//         )
//     );

//     return routine;
// }

// }
    

