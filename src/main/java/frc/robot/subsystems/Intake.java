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
import frc.lib.Colors.Colors;

// * Algae HSV -> 165, 51, 80 *
// * Coral HSV -> 27, 4, 87 *

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

        sensor = new Canandcolor(1);
        settings = new CanandcolorSettings();

        settings.setColorFramePeriod(0.040);
        settings.setLampLEDBrightness(1.0);
        settings.setAlignColorFramesToIntegrationPeriod(true);
        settings.setProximityIntegrationPeriod(ProximityPeriod.k20ms);

        motor.getConfigurator().apply(config);
        sensor.setSettings(settings);

    }

    public boolean hasAlgae(){
        if (getObjectDistance() >= 0.9 && getObjectDistance() <= 0.99){
            if (getColor().toString().equals(Colors.kAlgae.toString())) {
                return true;
            } else {
            return false;
            }
        } else {
            return false;
        }
    }

    public boolean hasCoral(){
        if (getObjectDistance() >= 0.9 && getObjectDistance() <= 0.99){
            if (getColor().toString().equals(Colors.kCoral.toString())) {
                return true;
            } else {
            return false;
            }
        } else {
            return false;
        }
    }

    public double getObjectDistance() {
        return sensor.getProximity();
    }

    public ColorData getColor() {
        return sensor.getColor();
    }

    public void intake() {
        motor.set(0.5);
    }

    public void outtake() {
        motor.set(-0.5);
    }

    public void stop() {
        motor.set(0.0);
    }

}