package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  AddressableLED led = new AddressableLED(0);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(33);

  private static final Distance ledSpacing = Meters.of(1.0 / 66.0);

  public final LEDPattern blue = LEDPattern.solid(Color.kBlue);
  public final LEDPattern red = LEDPattern.solid(Color.kRed);
  public final LEDPattern green = LEDPattern.solid(Color.kGreen);
  public final LEDPattern cyan = LEDPattern.solid(Color.kCyan);
  public final LEDPattern orange = LEDPattern.solid(Color.kOrange);
  public final LEDPattern purple = LEDPattern.solid(Color.kPurple);
  public final LEDPattern yellow = LEDPattern.solid(Color.kYellow);
  public final LEDPattern white = LEDPattern.solid(Color.kWhite);
  public final LEDPattern black = LEDPattern.solid(Color.kBlack);

  public final LEDPattern left = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kWhite, Color.kBlue);
  public final LEDPattern right = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kWhite, Color.kRed);
  public final LEDPattern leftCoral = 
      left.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(30), ledSpacing);
  public final LEDPattern rightCoral =
      right.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(30), ledSpacing);

  public final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
  public final LEDPattern scrollngRainbow =
      rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing);

  public LED() {

    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer);
    led.start();
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(ledBuffer));
  }

  public void setPattern(LEDPattern pattern) {
    pattern.applyTo(ledBuffer);
  }

  @Override
  public void periodic() {
    led.setData(ledBuffer);
    led.start();

    DogLog.log("LED/LED Pattern", ledBuffer.getLED(0).toString());
  }
}