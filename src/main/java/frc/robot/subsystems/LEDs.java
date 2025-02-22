package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

  private final AddressableLED leds = new AddressableLED(0);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(19);

  private static final Distance ledSpacing = Meters.of(1 / 38);

  private final LEDPattern blue = LEDPattern.solid(Color.kSteelBlue);
  private final LEDPattern raindow = LEDPattern.rainbow(255, 188);
  private final LEDPattern scrollRainbow =
      raindow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing);

  public LEDs() {
    leds.setLength(buffer.getLength());

    leds.setData(buffer);
    leds.start();
  }

  public void blue() {
    blue.applyTo(buffer);
  }

  public void rainbow() {
    scrollRainbow.applyTo(buffer);
  }

  @Override
  public void periodic() {
    leds.setData(buffer);
  }
}
