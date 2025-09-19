package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private ElevatorStates targetLevel = ElevatorStates.STOW;
    public ElevatorStates driverDesiredElevatorStates;

    private static Elevator instance;

    private final TrapezoidProfile elevatorProfile;

    private TrapezoidProfile.State elevatorGoal;
    private TrapezoidProfile.State elevatorCurrentPoint;

    public static Elevator getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Elevator instance not set");
        }
        return instance;
    }

    public void setTargetState(ElevatorStates state) {
        targetLevel = state;
    }

    public double getPosition() {
        return this.io.getPosition();
    }

    public static Elevator setInstance(ElevatorIO io) {
        instance = new Elevator(io);
        return instance;
    }

    private Elevator(ElevatorIO io) {
        this.io = io;

        this.elevatorProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(900, 1300));
        this.elevatorCurrentPoint = new TrapezoidProfile.State(getPosition(), 0);
    }

    public ElevatorIO getElevatorIo() {
        return io;
    }

    @Override
    public void periodic() {

        io.updateInputs(inputs);
        elevatorCurrentPoint = elevatorProfile.calculate(Constants.GeneralConstants.LOOP_TIME, elevatorCurrentPoint, elevatorGoal);
        manualSetTargetPosistion(elevatorCurrentPoint.position);

        Logger.processInputs("Elevator", inputs);
    }

    public void setState(ElevatorStates state) {
        if (state.position < 0)
            state.position = 0;

        this.elevatorGoal = new TrapezoidProfile.State(state.position, 0);

        // io.setPosition(state.position);
        targetLevel = state;
    }

    public ElevatorStates getTargetState() {

        return targetLevel;
    }

    public void manualSetTargetPosistion(double position) {
        if (position < 0)
            position = 0;
        io.setPosition(position);
    }

    public boolean isAtState(ElevatorStates state, double tolerance) {
        return MathUtil.isNear(this.inputs.leftPositionRotations, state.position, tolerance)
                || MathUtil.isNear(
                        this.inputs.rightPositionRotations, state.position, tolerance);
    }

    public boolean isAtState(double tolerance) {
        return MathUtil.isNear(this.inputs.leftPositionRotations, targetLevel.position, tolerance)
                || MathUtil.isNear(
                        this.inputs.rightPositionRotations, targetLevel.position, tolerance);
    }

    public void updateSim() {
        io.updateSim();
    }
}