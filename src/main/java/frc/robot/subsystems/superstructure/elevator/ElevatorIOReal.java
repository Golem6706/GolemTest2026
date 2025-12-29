package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

public class ElevatorIOReal implements ElevatorIO {
    // Hardware
    private final TalonFX elevatorTalon;
    private final TalonFX elevatorFollowerTalon;

    // CTRE Status Signals
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Current> motorSupplyCurrent;
    private final StatusSignal<Voltage> motorOutputVoltage;
    private final StatusSignal<Temperature> motorTemperature;

    public ElevatorIOReal() {
        this.elevatorTalon = new TalonFX(HARDWARE_CONSTANTS.ELEVATOR_MOTOR_ID());
        this.elevatorFollowerTalon = HARDWARE_CONSTANTS.ELEVATOR_FOLLOWER_ID().isPresent()
                ? new TalonFX(HARDWARE_CONSTANTS.ELEVATOR_FOLLOWER_ID().getAsInt())
                : null;
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerTime(OVERHEAT_PROTECTION_TIME)
                .withSupplyCurrentLowerLimit(OVERHEAT_PROTECTION_CURRENT);
        this.elevatorTalon.getConfigurator().apply(currentLimitsConfigs);
        if (this.elevatorFollowerTalon != null) {
            this.elevatorFollowerTalon.getConfigurator().apply(currentLimitsConfigs);
            this.elevatorFollowerTalon
                    .getConfigurator()
                    .apply(new MotorOutputConfigs()
                            .withNeutralMode(NeutralModeValue.Brake)
                            .withInverted(
                                    HARDWARE_CONSTANTS.ELEVATOR_FOLLOWER_INVERTED()
                                            ? InvertedValue.Clockwise_Positive
                                            : InvertedValue.CounterClockwise_Positive));
        }
        this.elevatorTalon
                .getConfigurator()
                .apply(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(
                                HARDWARE_CONSTANTS.ELEVATOR_MOTOR_INVERTED()
                                        ? InvertedValue.Clockwise_Positive
                                        : InvertedValue.CounterClockwise_Positive));

        this.motorPosition = elevatorTalon.getPosition();
        this.motorVelocity = elevatorTalon.getVelocity();
        this.motorSupplyCurrent = elevatorTalon.getSupplyCurrent();
        this.motorOutputVoltage = elevatorTalon.getMotorVoltage();
        this.motorTemperature = elevatorTalon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0, motorPosition, motorVelocity, motorSupplyCurrent, motorOutputVoltage, motorTemperature);

        elevatorTalon.optimizeBusUtilization();
        elevatorTalon.setPosition(0);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        StatusCode statusCode = BaseStatusSignal.refreshAll(
                motorPosition, motorVelocity, motorSupplyCurrent, motorOutputVoltage, motorTemperature);

        inputs.hardwareConnected = statusCode.isOK();
        inputs.encoderAngleRad = Units.rotationsToRadians(motorPosition.getValueAsDouble());
        inputs.encoderVelocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
        inputs.motorSupplyCurrentAmps = motorSupplyCurrent.getValueAsDouble();
        inputs.motorOutputVolts = motorOutputVoltage.getValueAsDouble();
        inputs.motorTemperatureCelsius = motorTemperature.getValueAsDouble();
    }

    private VoltageOut voltageOut = new VoltageOut(Volts.zero());

    @Override
    public void setMotorOutput(double volts) {
        voltageOut.withOutput(volts);
        elevatorTalon.setControl(voltageOut);
        if (elevatorFollowerTalon != null) elevatorFollowerTalon.setControl(voltageOut);
    }

    @Override
    public void setMotorBrake(boolean brakeModeEnable) {
        NeutralModeValue value = brakeModeEnable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        elevatorTalon.setNeutralMode(value);
        elevatorFollowerTalon.setNeutralMode(value);
    }
}
