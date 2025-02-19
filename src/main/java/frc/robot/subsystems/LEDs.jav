package frc.robot.subsystems;

public class LEDs extends SubsystemBase {

    private final led = new AddressableLED(0);
    private final ledBuffer = new AddressableLEDBuffer(12);

    private static final Distance ledSpacing = Meters.of(1.0 / 120.0)

    private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    private final LEDPattern scrollngRainbow = 
        rainbow.srcollAtAbsoluteSpeed(speedMetersPerSecond.of(1), ledSpacing);

    public LEDs() {

        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
        led.start();

    }

    public void rainbow() {
        scrollngRainbow.applyTo(ledBuffer);
    }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
    }
}