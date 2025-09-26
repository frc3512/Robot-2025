package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.ArmIO.ArmIOInputs;

public class Arm extends SubsystemBase{

    private final ArmIO io;
    private final ArmIOInputs inputs = new ArmIOInputs();

    private ArmStates targetLevel = ArmStates.STOW;

    public static Arm instance;

    private final TrapezoidProfile armProfile;

    private TrapezoidProfile.State armGoal;
    private TrapezoidProfile.State armCurrentPoint;

    public static Arm getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Arm instance not set");
        }
        return instance;
    }

    public void setTargetState(ArmStates newState) {
        targetLevel = newState;
    }

    public double getPosition() {
        return inputs.position;
    }

    public static Arm setInstance(ArmIO io) {
        instance = new Arm(io);
        return instance;
    }

    public Arm(ArmIO io) {
        this.io = io;

        // * TUNE THESE VALUES *
        this.armProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(200, 500));
        this.armCurrentPoint = new TrapezoidProfile.State(getPosition(), 0);
    }

    public ArmIO getArmIO() {
        return io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        armCurrentPoint = 
            armProfile.calculate(Constants.GeneralConstants.LOOP_TIME, armCurrentPoint, armGoal);
        manualSetPosition(armCurrentPoint.position);

        Logger.processInputs("Arm", (LoggableInputs) inputs);
    }

    public void setState(ArmStates state) {
        if (state.position < 0) state.position = 0;

        this.armGoal = new TrapezoidProfile.State(targetLevel.position, 0);
        targetLevel = state;
    }

    public boolean atSetpoint(ArmStates state) {
        return io.atSetpoint(state);
    }

    public ArmStates getTargetState() {
        return targetLevel;
    }

    public void manualSetPosition(double position) {
        if (position < 0) position = 0;

        io.setPosition(position);
    }

    public boolean isAtState(double tolerance) {
        return MathUtil.isNear(this.inputs.position, targetLevel.position, tolerance);
    }

    public void updateSim() {
        io.updateSim();
    }
    
}
