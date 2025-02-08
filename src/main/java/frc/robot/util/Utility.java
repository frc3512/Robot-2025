package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Utility {

  public static Distance rotationsToMeters(
      Angle rotations, Distance wheelRadius, double gearRatio) {

    /* Apply gear ratio to input rotations */
    var gearedRadians = rotations.in(Radians) / gearRatio;

    /* Then multiply the wheel radius by radians of rotation to get distance */
    return wheelRadius.times(gearedRadians);
  }

  public static Angle metersToRotations(Distance meters, Distance wheelRadius, double gearRatio) {
    /* Divide the distance by the wheel radius to get radians */
    var wheelRadians = meters.in(Meters) / wheelRadius.in(Meters);

    /* Then multiply by gear ratio to get rotor rotations */
    return Radians.of(wheelRadians * gearRatio);
  }
}
