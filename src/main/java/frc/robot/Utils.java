package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Utils { 
    
        public static Distance rotationsToMeters(Angle rotations) {
            /* Apply gear ratio to input rotations */
            var gearedRadians = rotations.in(Radians) / this.kGearRatio;
            /* Then multiply the wheel radius by radians of rotation to get distance */
            return this.kWheelRadius.times(gearedRadians);
        }
    
        public static Angle metersToRotations(Distance wheelRadius) {
            /* Divide the distance by the wheel radius to get radians */
            var wheelRadians = meters.in(Meters) / this.kWheelRadius.in(Meters);
        /* Then multiply by gear ratio to get rotor rotations */
        return Radians.of(wheelRadians * this.kGearRatio);
    }

}
