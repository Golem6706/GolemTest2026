// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.constants.DriveTrainConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.IO.CanBusIOReal;
import frc.robot.subsystems.drive.IO.GyroIOPigeon2;
import frc.robot.subsystems.drive.IO.GyroIOSim;
import frc.robot.subsystems.drive.IO.ModuleIOSim;
import frc.robot.subsystems.drive.IO.ModuleIOTalon;
import frc.robot.utils.MapleJoystickDriveInput;

import java.text.BreakIterator;
import java.util.Optional;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.utils.FieldMirroringUtils;
import org.ironmaple.utils.mathutils.MapleCommonMath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public static final boolean SIMULATE_AUTO_PLACEMENT_INACCURACY = true;
  
  // PDP for AdvantageKit logging
  public final LoggedPowerDistribution powerDistribution;

  // Subsystems
  public final SwerveDrive drive;


  // Controller
  public final DriverMap driver = new DriverMap.LeftHandedXbox(0);

  // private final LoggedDashboardChooser<Auto> autoChooser;
  private final SendableChooser<Supplier<Command>> testChooser;

  // private final SendableChooser<Supplier<Command>> testChooser;


  // Simulated drive
  private final SwerveDriveSimulation driveSimulation;

  private final Field2d field = new Field2d();
  // public final Trigger isAlgaeMode;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Robot.CURRENT_ROBOT_MODE) {
      case REAL ->{
        driveSimulation = null;
        powerDistribution = LoggedPowerDistribution.getInstance(1, PowerDistribution.ModuleType.kRev);
        
        // CTRE Chassis
        drive = new SwerveDrive(
          SwerveDrive.DriveType.CTRE_TIME_SYNCHRONIZED,
          new GyroIOPigeon2(TunerConstants.DrivetrainConstants), 
          new CanBusIOReal(TunerConstants.kCANBus),
          new ModuleIOTalon(TunerConstants.FrontLeft, "FrontLeft"),
          new ModuleIOTalon(TunerConstants.FrontRight, "FrontRight"),
          new ModuleIOTalon(TunerConstants.BackLeft, "BackLeft"),
          new ModuleIOTalon(TunerConstants.BackRight, "BackRight"));

        // Adding Vision, arm, elevator, coralHolder initialization
      }

      case SIM ->{
        SimulatedArena.overrideSimulationTimings(
            Seconds.of(Robot.defaultPeriodSecs), DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD);
            
        this.driveSimulation = new SwerveDriveSimulation(
            DriveTrainSimulationConfig.Default()
              .withRobotMass(DriveTrainConstants.ROBOT_MASS)
              .withBumperSize(DriveTrainConstants.BUMPER_LENGTH, DriveTrainConstants.BUMPER_WIDTH)
              .withTrackLengthTrackWidth(
                    DriveTrainConstants.TRACK_LENGTH, DriveTrainConstants.TRACK_WIDTH)
              .withSwerveModule(new SwerveModuleSimulationConfig(
                    DriveTrainConstants.DRIVE_MOTOR_MODEL, 
                    DriveTrainConstants.STEER_MOTOR_MODEL, 
                    DriveTrainConstants.DRIVE_GEAR_RATIO, 
                    DriveTrainConstants.STEER_GEAR_RATIO, 
                    DriveTrainConstants.DRIVE_FRICTION_VOLTAGE, 
                    DriveTrainConstants.STEER_FRICTION_VOLTAGE, 
                    DriveTrainConstants.WHEEL_RADIUS, 
                    DriveTrainConstants.STEER_INERTIA, 
                    DriveTrainConstants.WHEEL_COEFFICIENT_OF_FRICTION))
              .withGyro(DriveTrainConstants.gyroSimulationFactory),
           new Pose2d(3, 3, new Rotation2d()));

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        powerDistribution = LoggedPowerDistribution.getInstance();

        // Sim robot, instantiate physics sim IO implementations
        final ModuleIOSim frontLeft = new ModuleIOSim(driveSimulation.getModules()[0]),
                  frontRight = new ModuleIOSim(driveSimulation.getModules()[1]),
                  backLeft = new ModuleIOSim(driveSimulation.getModules()[2]),
                  backRight = new ModuleIOSim(driveSimulation.getModules()[3]);

        final GyroIOSim gyroIOSim = new GyroIOSim(driveSimulation.getGyroSimulation());

        drive = new SwerveDrive(
                  SwerveDrive.DriveType.GENERIC, 
                  gyroIOSim,
                  (canBusInput) -> {}, 
                  frontLeft, 
                  frontRight, 
                  backLeft, 
                  backRight);
         SimulatedArena.getInstance().resetFieldForAuto();
        // Adding Vision, arm, elevator, coralHolder initialization

      }
    
      default ->{
        this.driveSimulation = null;

        powerDistribution = LoggedPowerDistribution.getInstance();

        // Replayed robot, disable IO implementations
        drive = new SwerveDrive
                  (SwerveDrive.DriveType.GENERIC, 
                  (canBusInputs) -> {},
                  (inputs) -> {}, 
                  (inputs) -> {}, 
                  (inputs) -> {}, 
                  (inputs) -> {}, 
                  (inputs) -> {});
      }
    }

    // SmartDashboard.putData("Select Test", testChooser = buildTestChooser());

    SmartDashboard.putData("Field", field);
    SmartDashboard.putData("Selecte Test By Matt", testChooser = buidTestChooser());

    // Configure the trigger bindings
    configureButtonBindings();
  }


  private SendableChooser<Supplier<Command>> buidTestChooser() {
    final SendableChooser<Supplier<Command>> testChooser = new SendableChooser<>();
    testChooser.setDefaultOption("SwerveTest", Commands::none);
    testChooser.addOption(
      "Drive SysId - Quasistatic - Forward", () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    testChooser.addOption(
      "Drive SysId - Quasistatic - Reverse", () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    testChooser.addOption(
      "Drive SysId - Dynamic - Forward", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    testChooser.addOption(
      "Drive SysId - Dynamic - Reverse", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    return testChooser;
  }



  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public void configureButtonBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Joystick drive command
    final MapleJoystickDriveInput driveInput = driver.getDriveInput();
    IntSupplier pov = () -> -1;
    final JoystickDrive joystickDrive = new JoystickDrive(driveInput, () -> true , pov, drive);
    drive.setDefaultCommand(joystickDrive);
    JoystickDrive.instance = Optional.of(joystickDrive);

    // reset gyro heading manually in case the vision does not work
    driver.resetOdometryButton()
        .onTrue(Commands.runOnce(
                        () -> drive.setPose(new Pose2d(
                                                drive.getPose().getTranslation(), 
                                                
                                                FieldMirroringUtils.getCurrentAllianceDriverStationFacing())),
                        drive)
            .ignoringDisable(true));

    // Lock chassis with x-formation
    driver.lockChassisWithXFormatButton().whileTrue(drive.lockChassisWithXFormation());

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public Command getTestCommand() {
    return testChooser.getSelected().get();
  }

  public void updateFieldSimAndDisplay() {
    if (driveSimulation == null) return;
    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput("FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    Logger.recordOutput("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    
  }
}
