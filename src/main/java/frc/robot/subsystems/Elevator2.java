// package frc.robot.subsystems;

// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import dev.doglog.DogLog;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Elevator2 extends SubsystemBase {
//     private final TalonFX frontMotor = new TalonFX(Constants.ElevatorConstants.frontMotorID);
//     private final TalonFX backMotor = new TalonFX(Constants.ElevatorConstants.backMotorID);

//     public String scoringLevel = "stow";

//     public boolean scoringl2 = false;
//     public boolean scoringl3 = false;
//     public boolean scoringl4 = false;

//     private final ProfiledPIDController elevatorPID =
//         new ProfiledPIDController(
//             Constants.ElevatorConstants.kP,
//             Constants.ElevatorConstants.kI,
//             Constants.ElevatorConstants.kD,
//             Constants.ElevatorConstants.constraints);

//     public Elevator2() {
//         frontMotor.setNeutralMode(NeutralModeValue.Brake);
//         backMotor.setNeutralMode(NeutralModeValue.Brake);

//         elevatorPID.setTolerance(Constants.ElevatorConstants.tolerance);

//         backMotor.setControl(new Follower(frontMotor.getDeviceID(), false));

//         frontMotor.setPosition(0.0000); // Zero the front motor position

//         setLevel("stow"); // Set initial position to stow
//     }

//     public void setLevel(String level) {
//         double setpoint = Constants.ElevatorConstants.stowPos; // Default to stow position
//         switch (level) {
//             case "stow":
//                 setpoint = Constants.ElevatorConstants.stowPos;
//                 break;
//             case "hp":
//                 setpoint = Constants.ElevatorConstants.hpPos;
//                 break;
//             case "l1":
//                 setpoint = Constants.ElevatorConstants.l1Pos;
//                 break;
//             case "l2":
//                 setpoint = Constants.ElevatorConstants.l2Pos;
//                 break;
//             case "l3":
//                 setpoint = Constants.ElevatorConstants.l3Pos;
//                 break;
//             case "l4":
//                 setpoint = Constants.ElevatorConstants.l4Pos;
//                 break;
//             case "a1":
//                 setpoint = Constants.ElevatorConstants.a1Pos;
//                 break;
//             case "a2":
//                 setpoint = Constants.ElevatorConstants.a2Pos;
//                 break;
//             case "aStow":
//                 setpoint = Constants.ElevatorConstants.aStowPos;
//                 break;
//             default:
//                 setpoint = Constants.ElevatorConstants.stowPos; // Fallback to stow if unknown
// level
//                 break;
//         }
//         frontMotor.setVoltage(MathUtil.clamp(elevatorPID.calculate(getElevatorPos(), setpoint),
// 0.5, 47));

//         DogLog.log("Elevator/Elevator Setpoint", level);
//     }

//     public Command selectScoringLevel(String level) {
//         return Commands.runOnce(() -> scoringLevel = level);
//     }

//     public boolean isAtSetpoint() {
//         return elevatorPID.atGoal();
//     }
//     public double getElevatorPos() {
//         return frontMotor.getPosition().getValueAsDouble();
//     }

//     @Override
//     public void periodic() {
//         // PID Info
//         DogLog.log("Elevator/Elevator Position", getElevatorPos());
//         DogLog.log("Elevator/Elevator Setpoint", elevatorPID.getSetpoint().position);
//         DogLog.log("Elevator/Front Motor Voltage",
// frontMotor.getMotorVoltage().getValueAsDouble());

//         // General Info
//         DogLog.log("Elevator/Front Motor Temp", frontMotor.getDeviceTemp().getValueAsDouble());
//         DogLog.log("Elevator/Back Motor Temp", backMotor.getDeviceTemp().getValueAsDouble());
//         DogLog.log("Elevator/Scoring Level", scoringLevel);

//         // Scoring levels
//         DogLog.log("Elevator/Scoring l4", scoringl4);
//         DogLog.log("Elevator/Scoring l3", scoringl3);
//         DogLog.log("Elevator/Scoring l2", scoringl2);
//     }
// }
