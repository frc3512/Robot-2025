// package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
// import edu.wpi.first.wpilibj.DigitalInput;
// import frc.lib.command.ProfiledPIDSubsystem;
// import frc.robot.Constants;

// public class Reeftake extends ProfiledPIDSubsystem {

//   private final TalonFX reefAlgaePivotMotor = new TalonFX(Constants.ReeftakeConstants.pivotMotorID);
//   private final TalonFX intakeMotor = new TalonFX(Constants.ReeftakeConstants.intakeMotorID);

//   private final DigitalInput hasCoral =
//       new DigitalInput(Constants.ReeftakeConstants.digialInputChannel);

//   boolean coralIn = false;
//   boolean shouldScoreCoral = false;

//   public Reeftake() {
//     super(
//         new ProfiledPIDController(
//             Constants.ReeftakeConstants.kP,
//             Constants.ReeftakeConstants.kI,
//             Constants.ReeftakeConstants.kD,
//             Constants.ReeftakeConstants.constraints));
//     getController().setTolerance(Constants.ReeftakeConstants.tolerance);

//     reefAlgaePivotMotor.setNeutralMode(NeutralModeValue.Brake);
//     intakeMotor.setNeutralMode(NeutralModeValue.Brake);
//   }

//   public void coralLogic() {
//     if (hasCoral.get()) {
//       coralIn = true;
//     } else {
//       coralIn = false;
//     }
//   }

//   public void coralIntake() {
//     if (!coralIn) {
//       intakeMotor.set(0.5);
//     } else {
//       intakeMotor.set(0);
//     }
//   }

//   public void coralShoot() {
//     intakeMotor.set(0.8);
//     // check if coral is in before shooting
//     // something like this
//     /*
//     if (coralIn) {

//         intakeMotor.set(0.8);

//     } else {

//         intakeMotor.set(0);

//     }
//     */
//   }

//   public void reefAlgaeIntake() {
//     intakeMotor.set(0.5);
//   }

//   public void reefAlgaeOuttake() {
//     intakeMotor.set(-0.5);
//   }

//   public void reefAlgaeStop() {
//     intakeMotor.set(0);
//   }

//   @Override
//   protected void useOutput(double output, State setpoint) {
//     reefAlgaePivotMotor.setVoltage(output);
//   }

//   @Override
//   protected double getMeasurement() {
//     return reefAlgaePivotMotor.getPosition().getValueAsDouble();
//   }
// }
