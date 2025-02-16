// package frc.robot.subsystems;

// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.lib.command.ProfiledPIDSubsystem;
// import frc.lib.util.Utility;
// import frc.robot.Constants;

// public class Elevator extends ProfiledPIDSubsystem {
//   private final TalonFX elevatorMotorRt =
//       new TalonFX(Constants.ElevatorConstants.elevatorMotorRtID);
//   private final TalonFX elevatorMotorLt =
//       new TalonFX(Constants.ElevatorConstants.elevatorMotorLtID);

//   boolean bypassStop = false;

//   public Elevator() {
//     super(
//         new ProfiledPIDController(
//             Constants.ElevatorConstants.kP,
//             Constants.ElevatorConstants.kI,
//             Constants.ElevatorConstants.kD,
//             Constants.ElevatorConstants.constraints));
//     getController().setTolerance(Constants.ElevatorConstants.tolerance);

//     elevatorMotorLt.setControl(new Follower(elevatorMotorRt.getDeviceID(), false));

//     elevatorMotorRt.setNeutralMode(NeutralModeValue.Brake);
//     elevatorMotorLt.setNeutralMode(NeutralModeValue.Brake);
//   }

//   public void elevatorUp() {
//     elevatorMotorLt.set(0.5);
//     elevatorMotorRt.set(0.5);
//   }

//   public void elevatorDown() {
//     elevatorMotorLt.set(-0.5);
//     elevatorMotorRt.set(-0.5);
//   }

//   public void elevatorStop() {
//     elevatorMotorLt.set(0.0);
//     elevatorMotorRt.set(0.0);
//     disable();
//   }

//   public void stow() {
//     setElevatorGoal(0.0);
//     enable();
//   }

//   public void hp() {
//     setElevatorGoal(0.2768);
//     enable();
//   }

//   public void l1() {
//     setElevatorGoal(0.2895);
//     enable();
//   }

//   public void l2() {
//     setElevatorGoal(0.5752);
//     enable();
//   }

//   public void l3() {
//     setElevatorGoal(0.9499);
//     enable();
//   }

//   public void l4() {
//     setElevatorGoal(1.5214);
//     enable();
//   }

//   public void a1() {
//     setElevatorGoal(0.2895);
//     enable();
//   }

//   public void a2() {
//     setElevatorGoal(0.7403);
//     enable();
//   }

//   public void setElevatorGoal(double targetGoalMeters) {
//     setGoal(
//         Utility.metersToRotations(
//             targetGoalMeters,
//             Constants.ElevatorConstants.elevatorDrumRadiusMeters,
//             Constants.ElevatorConstants.elevatorGearRatio));
//   }

//   @Override
//   public void periodic() {
//     super.periodic();

//     SmartDashboard.putNumber(
//         "Elevator/ElevatorPos",
//         Utility.rotationsToMeters(
//             getMeasurement(),
//             Constants.ElevatorConstants.elevatorDrumRadiusMeters,
//             Constants.ElevatorConstants.elevatorGearRatio));
//   }

//   @Override
//   protected void useOutput(double output, State setpoint) {
//     elevatorMotorRt.setVoltage(output);
//   }

//   @Override
//   protected double getMeasurement() {
//     return elevatorMotorRt.getPosition().getValueAsDouble();
//   }
// }
