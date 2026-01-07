package frc.robot.subsystems.coralholder;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Millimeters;
import static frc.robot.subsystems.coralholder.CoralHolderConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder.TriFunction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.SuperStructureVisualizer;
import frc.robot.utils.AlertsManager;

public class CoralHolder extends SubsystemBase {
    //Hardware interface
    private final CoralHolderIO io;
    private final CoralHolderInputsAutoLogged inputs;

    //Triggers
    public final Trigger firstSensor;
    public final Trigger secondSensor;
    /** Whether the coral is anywhere inside the coralHolder (one of any sensor triggered) */
    public Trigger hasCoral;
    /** Whether the coral is in desired place (triggering both sensors) */
    public Trigger coralInPlace;

    //Alerts
    private final Alert motorHardwareFaultsAlert;
    private final Alert sensor1HardwareFaultsAlert;
    private final Alert sensor2HardwareFaultsAlert;

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<Rotation2d> armAngleSupplier;
    private final DoubleSupplier elevatorHeightSuppler;

    public CoralHolder(CoralHolderIO io, Supplier<Pose2d> robotPoseSupplier,
            Supplier<Rotation2d> armAngleSuppler,
            DoubleSupplier elevatorHeightSupplier) {
        this.io = io;
        inputs = new CoralHolderInputsAutoLogged();

        final double FIRST_SENSOR_THRESHOLD_MM = FIRST_SENSOR_THRESHOLD.in(Millimeters);
        final double SECOND_SENSOR_THRESHOLD_MM = SECOND_SENSOR_THRESHOLD.in(Millimeters);

        this.firstSensor = new Trigger(
            () -> inputs.firstSensorReadingValid && inputs.firstSensorDistanceMM < FIRST_SENSOR_THRESHOLD_MM);

        this.secondSensor = new Trigger(
            () -> inputs.secondSensorReadingValid && inputs.secondSensorDistanceMM < SECOND_SENSOR_THRESHOLD_MM);
        
        this.hasCoral = firstSensor.or(secondSensor);
        this.coralInPlace = firstSensor.and(secondSensor);
        this.robotPoseSupplier = robotPoseSupplier;
        this.armAngleSupplier = armAngleSuppler;
        this.elevatorHeightSuppler = elevatorHeightSupplier;

        this.motorHardwareFaultsAlert = 
                AlertsManager.create("Coral Holder roller motor hardware faults detected!", Alert.AlertType.kError);
        this.sensor1HardwareFaultsAlert = 
                AlertsManager.create("Coral Holder sensor 1 hardware faults detected!", Alert.AlertType.kError);
        this.sensor2HardwareFaultsAlert = 
                AlertsManager.create("Coral Holder sensor 2 hardware faults detected!", Alert.AlertType.kError);
    }

    public boolean hardwareOK() {
        return inputs.firstSensorConnected && inputs.secondSensorConnected && inputs.motorConnected;
    }

    private void setVoltage(double rollerVolts, double feederVolts) {
        if(!hardwareOK())
            rollerVolts = feederVolts =0;
        io.setRollerMotorOutput(rollerVolts);
        io.setCollectorMotorOutput(feederVolts);
    }

    @Override
    public void periodic() {
        //Update inputs from IO and AdvantageKit
        io.updateInputs(inputs);
        Logger.processInputs("CoralHolder", inputs);
        //Update alerts
        motorHardwareFaultsAlert.set(!inputs.motorConnected);
        sensor1HardwareFaultsAlert.set(!inputs.firstSensorConnected);
        sensor2HardwareFaultsAlert.set(!inputs.secondSensorConnected);

        Logger.recordOutput("CoralHolder/Has Coral", hasCoral.getAsBoolean());
        Logger.recordOutput("CoralHolder/Coral In Place", coralInPlace.getAsBoolean());
        Logger.recordOutput("CoralHolder/First Sensor", firstSensor.getAsBoolean());
        Logger.recordOutput("CoralHolder/Second Sensor", secondSensor.getAsBoolean());

        visualizeCoral();
    }

    /** VisualizeCoral will used in simulation */
    private void visualizeCoral() {
        String key = "CoralInRobot";
        Pose2d robotPose = robotPoseSupplier.get();
        double elevatorHeight = elevatorHeightSuppler.getAsDouble();
        Rotation2d armAngle = armAngleSupplier.get();
        
        // Add the following code when fisih the superStructure.java
        if(coralInPlace.getAsBoolean())
            SuperStructureVisualizer.visualizeCoralInCoralHolder(
                    key, robotPose, elevatorHeight, armAngle, Centimeters.of(8));
        else if(firstSensor.getAsBoolean())
            SuperStructureVisualizer.visualizeCoralInCoralHolder(
                    key, robotPose, elevatorHeight, armAngle, Centimeters.zero());
        else if(secondSensor.getAsBoolean())
            SuperStructureVisualizer.visualizeCoralInCoralHolder(
                    key, robotPose, elevatorHeight, armAngle, Centimeters.of(15));
        else Logger.recordOutput(key, new Pose3d(0, 0, -1, new Rotation3d()));     
    }


    /**
     * A sequence that intake a Coral.
     * <p>The sequence will roll the coral in and move it into the holding position (where both sensor1 and sensor2 are triggered)</p>
     */
    public Command intakeCoralSequence() {
        return Commands.sequence(
                //Run the rollers forward quickly until the coral hits the first sensor
                runVolts(VOLTAGE_SETTINGS.INTAKE_VOLTS(), 4.0)
                .until(firstSensor)
                .onlyIf(firstSensor.negate()),
                // Run the rollers backwards for 0.05 sencond for rapid brake
                runVolts(VOLTAGE_SETTINGS.BRAKE_VOLTS(), 1.0)
                .withTimeout(0.05)
                .onlyIf(firstSensor),
                // Next, run when the rollers forward slowly until the coral hits the second sensor
                runVolts(VOLTAGE_SETTINGS.SHUFFLE_VOLTS(), 2.0).until(coralInPlace))
                // Only run when the rollers are ont in place yet
                .onlyIf(coralInPlace.negate())
                // Stop the intake at the end of the command
                .finallyDo(() -> setVoltage(0.0, 0.0));
    }

    /** Move coral to the desired position */
    public Command moveCoralToPlace() {
        return runVolts(VOLTAGE_SETTINGS.SHUFFLE_VOLTS(), 1.0)
                .until(coralInPlace)
                .onlyIf(coralInPlace.negate().and(firstSensor))
                .finallyDo(() -> setVoltage(0.0, 0.0));
    }

    /**
     * Shuffles the coral sunch that
     * <p>This is used to move the coral to the appropriate position for scoring.
     * Namely moving coral back a little bit in the coralholder and then to shoot coral
     */
    public Command shuffleCoralSequence() {
        return Commands.sequence(
                        // If the coral is not in place (Triggering sensor 2) yet,
                        // running rollers slowly forward until it triggers sensor 2.
                        runVolts(VOLTAGE_SETTINGS.SHUFFLE_VOLTS(), 2.0)
                        .onlyIf(secondSensor.negate())
                        .until(secondSensor),
                        // Next, run the rollers slowly backwards until it dose not triggers sensor 2
                        runVolts(-VOLTAGE_SETTINGS.SHUFFLE_VOLTS(), 2.0).until(secondSensor.negate()))
                // Only shuffle the coral if there is a coral in the coralholder
                .onlyIf(hasCoral)
                .withTimeout(1.5)
                // Stop the intake at the end of the command
                .finallyDo(() -> setVoltage(0.0, 0.0));
    }

    public Command keepCoralShuffledForever() {
        return Commands.sequence(
                shuffleCoralSequence(),
                shuffleCoralSequence().onlyIf(secondSensor).repeatedly());
    }

    /** Score the Coral from the coralHolder */
    public Command scoreCoral(double timeOutSeconds) {
        return runVolts(VOLTAGE_SETTINGS.SHOOT_VOLTS(), 1)
                .finallyDo(() -> setVoltage(0.0, 0.0))
                .withTimeout(timeOutSeconds);
    }
    public Command runVolts(double rollerVolts, double feederVolts) {
        return run(() -> setVoltage(rollerVolts, feederVolts));
    }

    public Command runIdle() {
        return runVolts(0, 0);
    }
}
