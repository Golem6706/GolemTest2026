package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import java.util.OptionalInt;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.Robot;

public final class ElevatorConstants {
    //General Constants (Shared across all robots)
    public static final Distance HEIGHT_THRESHOLD_ENABLE_LOW_SPEED_MODE = Centimeters.of(45);
    public static final LinearVelocity ELEVATOR_MOVING_VELOCITY_THRESHOLD = MetersPerSecond.of(0.03);

    //Current Limits (Shared across all robots)
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(80);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(60);
    public static final Time OVERHEAT_PROTECTION_TIME = Seconds.of(1);
    public static final Current OVERHEAT_PROTECTION_CURRENT = Amps.of(40);

    //Elevator Hardware Constants
    public record ElevatorHardwareConstants(
        Distance CHAIN_LENGTH,              //The pitch of the chain
        int ELEVATOR_DRUM_WHEEL_TEETH,      //The number of sprocket wheel
        
        int ELEVATOR_STAGES,
        double ELEVATOR_GEARING_REDUCTION,
        DCMotor ELEVATOR_GEARBOX,
        Mass ELEVATOR_CARRIAGE_WEIGHT,
        Distance ELEVATOR_MAX_HEIGHT,
        int ELEVATOR_MOTOR_ID,
        boolean ELEVATOR_MOTOR_INVERTED,
        OptionalInt ELEVATOR_FOLLOWER_ID,
        boolean ELEVATOR_FOLLOWER_INVERTED ) {}

    public static final ElevatorHardwareConstants HARDWARE_CONSTANTS = new ElevatorHardwareConstants(
        Millimeters.of(9.525),
        12,
        3,
        5.0 * 60.0 / 36.0,
        DCMotor.getKrakenX60(2),
        Kilograms.of(4.0),
        Meters.of(1.82), // The max hight of the elevator 1.75
        15,
        false,
        OptionalInt.of(16),
        true);

    //Elevator PID Constants
    public record ElevatorPIDConstants(
        double kS,
        double kG,
        double kV,
        double kA,
        double kP_STRONG,
        double kP_WEAK,
        Voltage MAX_OUTPUT_VOLTAGE,
        Voltage MIN_OUTPUT_VOLTAGE,
        LinearVelocity VELOCITY_CONSTRAIN,
        LinearAcceleration ACCELERATION_CONSTRAIN,
        Distance TOLERANCE ) {}

    public static final ElevatorPIDConstants PID_CONSTANTS = new ElevatorPIDConstants(
         0.01,
         0.25,
         2.7,
         0.02, 
         6.0/0.2,
         3.0/0.2,
         Volts.of(12),
         Volts.of(-8),
         MetersPerSecond.of(3.2),
         MetersPerSecondPerSecond.of(8.0),
         Centimeters.of(2));

}
