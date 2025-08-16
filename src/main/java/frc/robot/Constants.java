package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static class GeneralConstants {
    public static final boolean tuningMode = false;
  }

  public static class VisionConstants {
    public static final String elevatorCam = "Arducam OV9281 3512 left";
    public static final String climberCam = "Arducam OV9281 3512 right";

    public static final Transform3d elevatorCamOffset =
        new Transform3d(
            Units.inchesToMeters(10.0),
            Units.inchesToMeters(6.0),
            Units.inchesToMeters(20),
            new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(270)));

    public static final Transform3d climberCamOffset =
        new Transform3d(
            Units.inchesToMeters(-10),
            Units.inchesToMeters(6.0),
            Units.inchesToMeters(8),
            new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(90)));

    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(2, 2, 4);
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final Constraints autoAimTranslationConstraints = new Constraints(2, 5);
    public static final Constraints autoAimRotationConstraints =
        new Constraints(Units.rotationsToRadians(3), Units.rotationsToRadians(10));

    public static final ProfiledPIDController xController_Position =
        new ProfiledPIDController(8, 0, 0, autoAimTranslationConstraints);
    public static final ProfiledPIDController yController_Position =
        new ProfiledPIDController(8, 0, 0, autoAimTranslationConstraints);
    public static final ProfiledPIDController thetaController_Position =
        new ProfiledPIDController(5, 0, 0, autoAimRotationConstraints);
  }
}
