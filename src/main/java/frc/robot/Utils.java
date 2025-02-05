package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.*;

public class Utils {
        
    public Distance rotationsToMeters(Angle rotations, Distance wheelRadius, double gearRatio) {
    
         /* Apply gear ratio to input rotations */
        var gearedRadians = rotations.in(Radians) / gearRatio;

        /* Then multiply the wheel radius by radians of rotation to get distance */
        return wheelRadius.times(gearedRadians);

    }
    
    public Angle metersToRotations(Distance wheelRadius, Double gearRatio) {

        /* Divide the distance by the wheel radius to get radians */
        var wheelRadians = Meters.in(Meters) / wheelRadius.in(Meters);

        /* Then multiply by gear ratio to get rotor rotations */
        return Radians.of(wheelRadians * gearRatio);

    }

}
