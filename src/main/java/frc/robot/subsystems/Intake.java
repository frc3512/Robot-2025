package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.reduxrobotics.sensors.canandcolor.ColorData;
import com.reduxrobotics.sensors.canandcolor.ProximityPeriod;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private final TalonFX motor;
    private final TalonFXConfiguration config;

    private final Canandcolor sensor;
    private final CanandcolorSettings settings;

    public Intake() {

        motor = new TalonFX(17);

        config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        sensor = new Canandcolor(31);
        settings = new CanandcolorSettings();

        settings.setColorFramePeriod(0.040);
        settings.setLampLEDBrightness(0.35);
        settings.setAlignColorFramesToIntegrationPeriod(true);
        settings.setProximityIntegrationPeriod(ProximityPeriod.k20ms);

        motor.getConfigurator().apply(config);
        sensor.setSettings(settings);

    }

    public double getObjectDistance() {
        return sensor.getProximity();
    }

    public ColorData getColor() {
        return sensor.getColor();
    }

    public double getR() {
        return sensor.getRed();
    }

    public double getG() {
        return sensor.getGreen();
    }

    public double getB() {
        return sensor.getBlue();
    }

    public void intakeCoral() {
        motor.set(-0.8);
    }

    public void intakeAlgae() {
        motor.set(-0.6);
    }

    public void outtake() {
        motor.set(0.5);
    }

    public void placeCoral() {
        motor.set(0.1);
    }

    public void hold() {
        motor.set(0.1);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.set(0.0);
    }

}