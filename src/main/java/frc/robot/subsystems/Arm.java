package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Notifier;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


/**
* Arm subsystem using SparkMAX with NEO motor
*/
@Logged(name = "Arm")
public class Arm extends SubsystemBase {
 // Constants
 private final int canID = 1;
 private final double gearRatio = 15;
  private final double kP = 1;
  private final double kI = 0;
  private final double kD = 0;
 private final double maxVelocity = 1; // rad/s
 private final double maxAcceleration = 1; // rad/s²
 private final boolean brakeMode = true;
 private final boolean enableStatorLimit = true;
 private final double statorCurrentLimit = 40;
 private final boolean enableSupplyLimit = false;
 private final double supplyCurrentLimit = 40;
 private final double armLength = 0.8382; // meters
 
 // Feedforward
 private final ArmFeedforward feedforward = new ArmFeedforward(
   0, // kS
   6.41, // kG
   1.19, // kV
   1.92  // kA
 );
 
 // Motor controller
 private final SparkMax motor;
private final RelativeEncoder encoder;
 
 // Control mode
 private enum ControlMode {
   OPEN_LOOP,
   POSITION,
   VELOCITY
 }
 private ControlMode currentControlMode = ControlMode.OPEN_LOOP;
 private double targetPosition = 0.0;
 private double targetVelocity = 0.0;
 
 // Profiled PID Controller
 private ProfiledPIDController profiledPIDController;
 private TrapezoidProfile.Constraints constraints;
 
 // Simulation
 private final SingleJointedArmSim armSim;
 
 /**
  * Creates a new Arm Subsystem.
  */
 public Arm() {
   // Initialize motor controller
   SparkMaxConfig motorConfig = new SparkMaxConfig();
motor = new SparkMax(canID, MotorType.kBrushless);
motorConfig.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

// Configure encoder
encoder = motor.getEncoder();
encoder.setPosition(0);


// Set current limits
 motorConfig.smartCurrentLimit(statorCurrentLimit);


// Save configuration
motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
   // Initialize simulation
   armSim = new SingleJointedArmSim(
     DCMotor.getNEO(1), // Motor type
     gearRatio,
     SingleJointedArmSim.estimateMOI(armLength, 5.896696), // Arm moment of inertia
     armLength, // Arm length (m)
     Units.degreesToRadians(0), // Min angle (rad)
     Units.degreesToRadians(1.5707963267948966), // Max angle (rad)
     true, // Simulate gravity
     Units.degreesToRadians(0) // Starting position (rad)
   );
   
   // Initialize ProfiledPIDController
   // Convert from radians to rotations for constraints
   double maxVelocityRotations = maxVelocity / (2.0 * Math.PI);
   double maxAccelerationRotations = maxAcceleration / (2.0 * Math.PI);
   
   constraints = new TrapezoidProfile.Constraints(maxVelocityRotations, maxAccelerationRotations);
   profiledPIDController = new ProfiledPIDController(kP, kI, kD, constraints);
 }
 
 /**
  * Control loop function that runs at a fixed frequency.
  * This is used for SparkMAX and SparkFlex controllers to implement
  * closed-loop control outside of the main robot loop.
  */
 private void controlLoopFn() {
   switch (currentControlMode) {
     case POSITION:
       double currentPos = getPosition();
       double output = profiledPIDController.calculate(currentPos, targetPosition);
       double velocity = profiledPIDController.getSetpoint().velocity;
       double angle = getPositionRadians();
       double feedforwardOutput = feedforward.calculate(angle, velocity);
       setVoltage(output + feedforwardOutput);
       break;
       
     case VELOCITY:
       double currentVel = getVelocity();
       double velOutput = profiledPIDController.calculate(currentVel, targetVelocity);
       double accel = profiledPIDController.getSetpoint().velocity - currentVel;
       double armAngle = getPositionRadians();
       double velFeedforwardOutput = feedforward.calculate(armAngle, targetVelocity, accel);
       
       // Apply the combined PID output and feedforward to the motor
       double velocityVoltage = velOutput + velFeedforwardOutput;
       motor.setVoltage(velocityVoltage);
       break;
       
     case OPEN_LOOP:
     default:
       // Do nothing, voltage is set directly
       break;
   }
 }
 
 /**
  * Clean up resources when the subsystem is destroyed.
  */
  public void close() {
     motor.close();
   }
 }
 
 /**
  * Update simulation and telemetry.
  */
 @Override
 public void periodic() {
   
   controlLoopFn();
 }
 
 /**
  * Update simulation.
  */
 @Override
 public void simulationPeriodic() {
   // Set input voltage from motor controller to simulation
   armSim.setInput(getVoltage());
   
   // Update simulation by 20ms
   armSim.update(0.020);
 }
 
 /**
  * Get the current position in the Rotations.
  * @return Position in Rotations
  */
  @Logged(name = "Position/Rotations")
public double getPosition() {
  // Rotations
  return encoder.getPosition() / gearRatio;
}


/**
 * Get the current velocity in rotations per second.
 * @return Velocity in rotations per second
 */
  @Logged(name = "Velocity")
public double getVelocity() {
  return encoder.getVelocity() / gearRatio / 60.0; // Convert from RPM to RPS
}

/**
 * Get the current applied voltage.
 * @return Applied voltage
 */
  @Logged(name = "Voltage")
public double getVoltage() {
  return motor.getAppliedOutput() * motor.getBusVoltage();
}

/**
 * Get the current motor current.
 * @return Motor current in amps
 */
  @Logged(name = "Current")
public double getCurrent() {
  return motor.getOutputCurrent();
}

/**
 * Get the current motor temperature.
 * @return Motor temperature in Celsius
 */
  @Logged(name = "Temperature")
public double getTemperature() {
  return motor.getMotorTemperature();
}
 
 /**
  * Set arm angle.
  * @param angleDegrees The target angle in degrees
  */
 public void setAngle(double angleDegrees) {
   setAngle(angleDegrees, 0);
 }
 
 /**
  * Set arm angle with acceleration.
  * @param angleDegrees The target angle in degrees
  * @param acceleration The acceleration in rad/s²
  */
 public void setAngle(double angleDegrees, double acceleration) {
   // Convert degrees to rotations
   double angleRadians = Units.degreesToRadians(angleDegrees);
   double positionRotations = angleRadians / (2.0 * Math.PI);
   
   // Use the ProfiledPIDController
   targetPosition = positionRotations;
   currentControlMode = ControlMode.POSITION;
   
   // If acceleration is specified, update constraints
   if (acceleration > 0) {
     double maxAccelRotations = acceleration / (2.0 * Math.PI);
     constraints = new TrapezoidProfile.Constraints(constraints.maxVelocity, maxAccelRotations);
     profiledPIDController.setConstraints(constraints);
   }
 }
 
 /**
  * Set arm angular velocity.
  * @param velocityDegPerSec The target velocity in degrees per second
  */
 public void setVelocity(double velocityDegPerSec) {
   setVelocity(velocityDegPerSec, 0);
 }
 
 /**
  * Set arm angular velocity with acceleration.
  * @param velocityDegPerSec The target velocity in degrees per second
  * @param acceleration The acceleration in degrees per second squared
  */
 public void setVelocity(double velocityDegPerSec, double acceleration) {
   // Convert degrees/sec to rotations/sec
   double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
   double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);
   
   // Use the ProfiledPIDController
   targetVelocity = velocityRotations;
   currentControlMode = ControlMode.VELOCITY;
   
   // If acceleration is specified, update constraints
   if (acceleration > 0) {
     double maxAccelRotations = Units.degreesToRadians(acceleration) / (2.0 * Math.PI);
     constraints = new TrapezoidProfile.Constraints(constraints.maxVelocity, maxAccelRotations);
     profiledPIDController.setConstraints(constraints);
   }
   
   // Apply velocity directly to the motor controller as well
   // This ensures immediate response while the control loop refines it
   double armAngle = getPositionRadians();
   double ffVolts = feedforward.calculate(armAngle, velocityRotations, 0);
   motor.setVoltage(ffVolts);
 }
 
 /**
  * Set motor voltage directly.
  * @param voltage The voltage to apply
  */
 public void setVoltage(double voltage) {
   currentControlMode = ControlMode.OPEN_LOOP;
   motor.setVoltage(voltage);
 }
 
 /**
  * Get the arm simulation for testing.
  * @return The arm simulation model
  */
 public SingleJointedArmSim getSimulation() {
   return armSim;
 }
 
 /**
  * Creates a command to set the arm to a specific angle.
  * @param angleDegrees The target angle in degrees
  * @return A command that sets the arm to the specified angle
  */
 public Command setAngleCommand(double angleDegrees) {
   return runOnce(() -> setAngle(angleDegrees));
 }
 
 /**
  * Creates a command to move the arm to a specific angle with a profile.
  * @param angleDegrees The target angle in degrees
  * @return A command that moves the arm to the specified angle
  */
 public Command moveToAngleCommand(double angleDegrees) {
   return run(() -> {
     // Just set the position and let the profiled controller handle it
     setAngle(angleDegrees);
   }).until(() -> {
     double currentAngle = Units.radiansToDegrees(getPositionRadians());
     return Math.abs(angleDegrees - currentAngle) < 2.0; // 2 degree tolerance
   }).finallyDo((interrupted) -> setVelocity(0));}
 
 /**
  * Creates a command to stop the arm.
  * @return A command that stops the arm
  */
 public Command stopCommand() {
   return runOnce(() -> setVelocity(0));
 }
 
 /**
  * Creates a command to move the arm at a specific velocity.
  * @param velocityDegPerSec The target velocity in degrees per second
  * @return A command that moves the arm at the specified velocity
  */
 public Command moveAtVelocityCommand(double velocityDegPerSec) {
   return run(() -> setVelocity(velocityDegPerSec));
 }