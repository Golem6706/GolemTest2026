package frc.robot.subsystems.coralholder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public final class CoralHolderConstants {
    public record HardwareConstants(
        int rollerMotorID,
        boolean rollerMotorInverted,
        int firstSeneorID,          //LaserCAN ID
        int secondSensorID,         //LaserCAN ID
        int[] feederMotorsIDs,      //Coral intake motors which is not contains in robot of team 6706
        boolean[] feederMotorsInverted
    ) {}

    public static final HardwareConstants HARDWARE_CONSTANTS = 
            new HardwareConstants(18, true, 0, 1, new int[0], new boolean[0]);

    /**
     * Several settings to set the coral holder and coral intake motor
     * @param INTAKE_VOLTS The volts of motor to intake coral
     * @param SHOOT_VOLTS The volts of motor to shoot coral
     * @param BRAKE_VOLTS The volts of motor to brake or hold coral
     * @param SHUFFLE_VOLTS The volts of motor to roll coral back a little bit in the coralholder
     */
    public record VoltageSettings(double INTAKE_VOLTS, double SHOOT_VOLTS, double BRAKE_VOLTS, double SHUFFLE_VOLTS){};
    public static final VoltageSettings VOLTAGE_SETTINGS = new VoltageSettings(4.0, 7.0, -2.0, 1);

    public static final Distance FIRST_SENSOR_THRESHOLD = Centimeters.of(5.0);
    public static final Distance SECOND_SENSOR_THRESHOLD = Centimeters.of(6.0);
    public static final Current ROLLERS_CURRENT_LIMIT = Amps.of(20);

    //Simulation Constants
    public static final Translation3d COLLECTOR_POSITION_ON_ROBOT = new Translation3d(-0.3, 0, 0.6);
    public static Translation3d COLLECTOR_RANGE = new Translation3d(0.2, 0.3, 0.2);

    public static final double COLLECTOR_TIME_SECONDS_AT_6V = 0.1;
    public static final double ROLLER_TIME_SECONDS_AT_6V = 0.2;
    public static final LinearVelocity CORAL_LAUNCHING_VELOCITY_6V = MetersPerSecond.of(3);

    public static final Distance CORAL_LENGHT_ON_ARM = Centimeters.of(35.2);
    public static final Rotation2d ARM_ANGLE_TO_CORAL_POSITION_ANGLE = Rotation2d.fromDegrees(-148);
    public static final Rotation2d ARM_PINPOINT_TO_CORAL_DIRECTION = Rotation2d.fromDegrees(-5);
}
