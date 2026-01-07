package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class SuperStructure {
    /**
    * Represents a pose of the super structure
    * <p> The pose of the super structure is the combination of the elevator height and arm position
    */
    public enum SuperStructurePose {

        //Highly used poses
        IDLE(0.05, Degrees.of(113));
        public final double elevatorHeightMeters;
        public final Angle armAngle;
        SuperStructurePose (double elevatorHeightMeters, Angle armAngle) {
            this.elevatorHeightMeters = elevatorHeightMeters;
            this.armAngle = armAngle;
        }
        

    }
}
