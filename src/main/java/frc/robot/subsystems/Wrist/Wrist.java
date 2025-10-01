package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist.WristIO.WristIOInputs;

public class Wrist extends SubsystemBase{

    private final WristIO io;
    private final WristIOInputs inputs = new WristIOInputs();

    private WristStates targetLevel = WristStates.VERTICAL;

    public static Wrist instance;

    private final TrapezoidProfile wristProfile;

    private TrapezoidProfile.State wristGoal;
    private TrapezoidProfile.State wristCurrentPoint;

    public static Wrist getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Wrist instance not set");
        }
        return instance;
    }

    public void setTargetState(WristStates newState) {
        targetLevel = newState;
    }

    public double getPosition() {
        return inputs.position;
    }

    public static Wrist setInstance(WristIO io) {
        instance = new Wrist(io);
        return instance;
    }

    public Wrist(WristIO io) {
        this.io = io;

        this.wristProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(50, 100));
        this.wristCurrentPoint = new TrapezoidProfile.State(getPosition(), 0);
    }

    public WristIO getWristIO() {
        return io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        wristCurrentPoint = 
            wristProfile.calculate(Constants.GeneralConstants.LOOP_TIME, wristCurrentPoint, wristGoal);
            
        manualSetPosition(wristCurrentPoint.position);

        Logger.processInputs("Arm", (LoggableInputs) inputs);
    }

    public void setState(WristStates state) {
        this.wristGoal = new TrapezoidProfile.State(targetLevel.position, 0);
        targetLevel = state;
    }

    public boolean atSetpoint() {
        return io.atSetpoint();
    }

    public WristStates getTargetState() {
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
