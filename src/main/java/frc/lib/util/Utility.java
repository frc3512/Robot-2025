package frc.lib.util;

public class Utility {
  public static double metersToRotations(
      double targetGoalMeters, double wheelRadiusMeters, double gearRatio) {
    return (targetGoalMeters / (2 * Math.PI * wheelRadiusMeters)) * gearRatio;
  }

  public static double rotationsToMeters(
      double targetGoalRotations, double wheelRadiusMeters, double gearRatio) {
    return (targetGoalRotations * 2 * Math.PI * wheelRadiusMeters) / gearRatio;
  }
}