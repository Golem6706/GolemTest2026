package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;

public final class ArmConstants {
    public static final Current ARM_CURRENT_LIMIT = Amps.of(30.0);
    public static final Voltage ARM_MAX_VOLTAGE = Volts.of(8.0);

    public record ArmHardwareConstants (
        Distance ARM_COM_LENGTH,
        Mass ARM_MASS,      // The weight of the arm
        DCMotor ARM_GEARBOX,    // The gearbox which contains several motors
        double ARM_GEARING_REDUCTION,
        Angle ARM_UPPER_HARD_LIMIT,
        Angle ARM_LOWER_HARD_LIMIT,
        Angle ABSOLUTE_ENCODER_READING_AT_UPPER_LIM,
        int ABSOLUTE_ENCONDER_CHANNEL,
        boolean ABSOLUTE_ENCONDER_INVERTED,
        int ARM_MOTOR_ID,
        boolean ARM_MOTOR_INVERTED) {}

    public static final ArmHardwareConstants HARDWARE_CONSTANTS =
        new ArmHardwareConstants(
            Centimeters.of(26.0),
            Kilograms.of(4.0),
            DCMotor.getKrakenX60(1),
            24.0 / 12.0 * 20.0,
            Degrees.of(136.0),
            Degrees.of(-54.0), 
            Rotation.of(0.11),
            0,
            true,
            19,
            false);


    public record ArmPIDConstants (
        double kS,
        double kG,
        double kV,
        double kA,
        double kP,
        AngularVelocity VELOCTIY_CONSTRAIN,
        AngularAcceleration ACCELERATION_CONSTRAIN,
        Angle tOLERANCE) {}

    public static final ArmPIDConstants PID_CONSTANTS = 
        new ArmPIDConstants(
            0.05,
            0.33,
            0.76,
            0.01,
            6.0 / Math.toRadians(30), 
            RotationsPerSecond.of(1),
            RotationsPerSecondPerSecond.of(5),
            Degrees.of(3));
}
