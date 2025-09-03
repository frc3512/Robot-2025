package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import edu.wpi.first.units.measure.*;

@SuppressWarnings("removal")
@Logged(name = "Elevator")
public class Elevator extends SubsystemBase {

  // Motor controller
  private final TalonFX frontMotor;
  private final TalonFX backMotor;

  private final PositionVoltage positionRequest;
  private final VelocityVoltage velocityRequest;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  private final double gearRatio = Constants.ElevatorConstants.gearRatio;

  private final double maxVelocity = Constants.ElevatorConstants.maxVelocity;
  private final double maxAcceleration = Constants.ElevatorConstants.maxAcceleration;

  private final boolean brakeMode = Constants.ElevatorConstants.brakeMode;

  private final double forwardSoftLimit = Constants.ElevatorConstants.forwardSoftLimit;
  private final double reverseSoftLimit = Constants.ElevatorConstants.reverseSoftLimit;

  private final boolean enableStatorLimit = Constants.ElevatorConstants.enableStatorLimit;
  private final int statorCurrentLimit = Constants.ElevatorConstants.statorCurrentLimit;
  private final boolean enableSupplyLimit = Constants.ElevatorConstants.enableSupplyLimit;
  private final double supplyCurrentLimit = Constants.ElevatorConstants.supplyCurrentLimit;

  private final double drumRadius = Constants.ElevatorConstants.drumRadius;

  private final double minheight = Constants.ElevatorConstants.minheight;
  private final double maxheight = Constants.ElevatorConstants.maxheight;

  // Feedforward
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
    0, // kS
    0.62, // kG
    3.41, // kV
    0.06  // kA
  );

  // Simulation
  private final ElevatorSim elevatorSim;

  public Elevator() {

    frontMotor = new TalonFX(Constants.ElevatorConstants.frontMotorID);
    backMotor = new TalonFX(Constants.ElevatorConstants.backMotorID);

    backMotor.setControl(
      new Follower(
        Constants.ElevatorConstants.frontMotorID, 
        false));

    // Create control requests
    positionRequest = new PositionVoltage(0).withSlot(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    // get status signals
    positionSignal = frontMotor.getPosition();
    velocitySignal = frontMotor.getVelocity();
    voltageSignal = frontMotor.getMotorVoltage();
    statorCurrentSignal = frontMotor.getStatorCurrent();
    temperatureSignal = frontMotor.getDeviceTemp();

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Configure PID for slot 0
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = Constants.ElevatorConstants.kP;
    slot0.kI = Constants.ElevatorConstants.kI;
    slot0.kD = Constants.ElevatorConstants.kD;

    // Set current limits
    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimit = statorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    // Set soft limits
    SoftwareLimitSwitchConfigs softLimits = config.SoftwareLimitSwitch;
      softLimits.ForwardSoftLimitThreshold = forwardSoftLimit;
      softLimits.ForwardSoftLimitEnable = true;
      softLimits.ReverseSoftLimitThreshold = reverseSoftLimit;
      softLimits.ReverseSoftLimitEnable = true;

    // Set brake mode
    config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Apply gear ratio
    config.Feedback.SensorToMechanismRatio = gearRatio;

    // Apply configuration
    frontMotor.getConfigurator().apply(config);
  
    // Reset encoder position
    frontMotor.setPosition(0);
    
    // Initialize simulation
    elevatorSim = new ElevatorSim(
      DCMotor.getKrakenX60(1), // Motor type
      gearRatio,
      5, // Carriage mass (kg)
      drumRadius, // Drum radius (m)
      0, // Min height (m)
      1.42, // Max height (m)
      true, // Simulate gravity
      0 // Starting height (m)
    );
    
  }
   
  /**
   * Update simulation and telemetry.
  */

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(positionSignal, velocitySignal, voltageSignal, statorCurrentSignal, temperatureSignal);
  }
  
  /**
   * Update simulation.
  */

  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    elevatorSim.setInput(getVoltage());
    
    // Update simulation by 20ms
    elevatorSim.update(0.020);

    // Convert meters to motor rotations
    double positionToRotations = (1 / (2.0 * Math.PI * drumRadius)) * gearRatio;
    double motorPosition = elevatorSim.getPositionMeters() * positionToRotations;
    double motorVelocity = elevatorSim.getVelocityMetersPerSecond() * positionToRotations;
    
    frontMotor.getSimState().setRawRotorPosition(motorPosition);
    frontMotor.getSimState().setRotorVelocity(motorVelocity);
    backMotor.getSimState().setRawRotorPosition(motorPosition);
    backMotor.getSimState().setRotorVelocity(motorVelocity);

  }
  
  /**
   * Get the current position in the Rotations.
   * @return Position in Rotations
  */

  @Logged(name = "Position/Rotations")
  public double getPosition() {
    // Rotations
      return positionSignal.getValueAsDouble();
  }

  
  /**
   * Get the current velocity in rotations per second.
   * @return Velocity in rotations per second
  */

  @Logged(name = "Velocity")
  public double getVelocity() {
    return velocitySignal.getValueAsDouble();
  }
  
  /**
   * Get the current applied voltage.
   * @return Applied voltage
  */

  @Logged(name = "Voltage")
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }
  
  /**
   * Get the current motor current.
   * @return Motor current in amps
  */

  @Logged(name = "Current")
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }
  
  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
  */

  @Logged(name = "Temperature")
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }
  
  /**
   * Set elevator position.
   * @param position The target position in meters
  */

  public void setPosition(double position) {
    setPosition(position, 0);
  }
  
  /**
   * Set elevator position with acceleration.
   * @param position The target position in meters
   * @param acceleration The acceleration in meters per second squared
  */

  public void setPosition(double position, double acceleration) {
    // Convert meters to rotations
    double positionRotations = position / (2.0 * Math.PI * drumRadius);
    double ffVolts = feedforward.calculate(getVelocity(), acceleration);
    
    frontMotor.setControl(positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
    backMotor.setControl(positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
  }

  /**
   * Set elevator velocity with acceleration.
   * @param velocity The target velocity in meters per second
   * @param acceleration The acceleration in meters per second squared
  */

  public void setVelocity(double velocity, double acceleration) {
    // Convert meters/sec to rotations/sec
    double velocityRotations = velocity / (2.0 * Math.PI * drumRadius);
    double ffVolts = feedforward.calculate(getVelocity(), acceleration);

    frontMotor.setControl(velocityRequest.withVelocity(velocityRotations).withFeedForward(ffVolts));
    backMotor.setControl(positionRequest.withPosition(velocityRotations).withFeedForward(ffVolts));
  } 

  /**
   * Set elevator velocity.
   * @param velocity The target velocity in meters per second
  */

  public void setVelocity(double velocity) {
    setVelocity(velocity, maxAcceleration);
  }
  
  /**
   * Set motor voltage directly.
   * @param voltage The voltage to apply
  */

  public void setVoltage(double voltage) {
    frontMotor.setVoltage(voltage);
    backMotor.setVoltage(voltage);
  }
  
  /**
   * Get the elevator simulation for testing.
   * @return The elevator simulation model
  */

  public ElevatorSim getSimulation() {
    return elevatorSim;
  }

  public double getMinHeightMeters() {
    return minheight;
  }

  public double getMaxHeightMeters() {
    return maxheight;
  }

  /**
   * Creates a command to set the elevator to a specific height.
   * @param heightMeters The target height in meters
   * @return A command that sets the elevator to the specified height
  */

  public Command setHeightCommand(double heightMeters) {
    return runOnce(() -> setPosition(heightMeters));
  }
  
  /**
   * Creates a command to move the elevator to a specific height with a profile.
   * @param heightMeters The target height in meters
   * @return A command that moves the elevator to the specified height
  */

  public Command setScoringLevel(double heightMeters) {
    return run(() -> {
      double currentHeight = getPosition() * (2.0 * Math.PI * drumRadius);
      double error = heightMeters - currentHeight;
      double velocity = Math.signum(error) * Math.min(Math.abs(error) * 2.0, maxVelocity);
      setVelocity(velocity);
    }).until(() -> {
      double currentHeight = getPosition() * (2.0 * Math.PI * drumRadius);
      return Math.abs(heightMeters - currentHeight) < 0.035; // cm tolerance
    });
  }
  
  /**
   * Creates a command to stop the elevator.
   * @return A command that stops the elevator
  */

  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }
  
  /**
   * Creates a command to move the elevator at a specific velocity.
   * @param velocityMetersPerSecond The target velocity in meters per second
   * @return A command that moves the elevator at the specified velocity
  */

  public Command moveAtVelocityCommand(double velocityMetersPerSecond) {
    return run(() -> setVelocity(velocityMetersPerSecond));
  }

}