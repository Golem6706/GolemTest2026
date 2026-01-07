package frc.robot.subsystems.superstructure.arm;

import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;

public interface ArmIO {
    
    final class ArmInputs implements LoggableInputs {
        /** The (optional) arm absolute enconder angle which is already calibrated. Empty if the absolute Enconder is disconnected. */
        public Optional<Rotation2d> absoluteEncoderAngle;
        /** Whether the CAN communications between the rio and the motor are good. */
        public boolean motorConnected;
        /** The relative encoder angle, measured by the relative encoder. Gearing is NOT considered. */
        public double relativeEncoderAngledRad;
        /** The relative encoder velocity, measured by the relative encoder. Gearing is NOT considered. */
        public double encoderVelocityRadPerSec;
        /** The supply current of the motor. */
        public double motorSupplyCurrentAmps;
        /** The actual output voltage of the motor. */
        public double motorOutputVolts;

        public ArmInputs() {
            this.absoluteEncoderAngle = Optional.empty();
            motorConnected = false;
            this.relativeEncoderAngledRad = 0.0;
            this.encoderVelocityRadPerSec = 0.0;
            this.motorSupplyCurrentAmps = 0.0;
            this.motorOutputVolts = 0.0;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("absoluteEncoderAnglePresent", absoluteEncoderAngle.isPresent());
            table.put("absoluteEncoderAngle", absoluteEncoderAngle.orElse(Rotation2d.kZero));
            table.put("motorConnected", motorConnected);
            table.put("relativeEncoerAngledRad", relativeEncoderAngledRad);
            table.put("encoderVelocityRadPerSec", encoderVelocityRadPerSec);
            table.put("motorSupplyCurrentAmps", motorSupplyCurrentAmps);
            table.put("motorOutputVolts", motorOutputVolts);
        }
        @Override
        public void fromLog(LogTable table) {
            boolean absoluteEnconderAnglePresent = table.get("absoluteEncoderAnglePresent",false);
            absoluteEncoderAngle = absoluteEnconderAnglePresent ? Optional.of(
                table.get("absoluteEncoderAngle", Rotation2d.kZero)) : Optional.empty();
            motorConnected = table.get("motorConnected", motorConnected);
            relativeEncoderAngledRad = table.get("relativeEncoerAngledRad", 0.0);
            encoderVelocityRadPerSec = table.get("encoderVelocityRadPerSec", 0.0);
            motorSupplyCurrentAmps = table.get("motorSupplyCurrentAmps", 0.0);
            motorOutputVolts = table.get("motorOutputVolts", 0.0);
        }
    }

    void updateInputs(ArmInputs armInputs);

    default void setMotorOutput(Voltage voltage) {}
    default void setMotorBrake(boolean brakeModeEnable) {}
}
