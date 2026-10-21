package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands.DriveCommands;
import frc.robot.commands.HoodZeroCommand.HoodZeroCommand;
import frc.robot.commands.IntakeCommand.IntakeCommand;
import frc.robot.commands.IntakeDeployZeroCommand.IntakeDeployZeroCommand;
import frc.robot.commands.ManualCommand.Manual;
import frc.robot.commands.ShootingCommand.Shooting;
import frc.robot.generated.TunerConstants;
import frc.robot.simulation.FuelSimulation;
import frc.robot.simulation.MapleSimArena;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.FeedPath.FeedPath;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.DashboardControls;
import frc.robot.util.LoggedTunableNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // D-pad up/down nudges Flywheel Speed Limit (%) by this much.
  private static final double FLYWHEEL_PERCENT_STEP = 1.0;

  // Subsystems
  private final Drive drive;

  // Touching each singleton here forces its class to initialize at boot, which runs the
  // constructor and flashes the motor config (gear ratio, PID, etc). Without an eager reference
  // these only initialize on first button press, so the config is never applied at deploy.
  private final Drum drum = Drum.mInstance;
  private final FeedPath feedPath = FeedPath.mInstance;
  private final Feeder feeder = Feeder.mInstance;
  private final Hood hood = Hood.mInstance;
  private final Indexer indexer = Indexer.mInstance;
  private final IntakeDeploy intakeDeploy = IntakeDeploy.mInstance;
  private final IntakeRoller intakeRoller = IntakeRoller.mInstance;
  private final LoggedTunableNumber retract = new LoggedTunableNumber("retract", 25);

  // Controller
  private final CommandXboxController controller = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        Drive.mInstance = drive;

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot: maple-sim arena + AdvantageKit drive IO (official hardware-abstraction path)
        var driveSimulation = MapleSimArena.getInstance().getDriveSimulation();
        FuelSimulation.configure(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]));
        Drive.mInstance = drive;
        drive.setPose(MapleSimArena.getInstance().getRobotPose());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        Drive.mInstance = drive;
        break;
    }
    Drive.mInstance = drive;

    // Demo robot: no autonomous routines, no vision — teleop only.

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * 0.9,
            () -> -controller.getLeftX() * 0.9,
            () -> -controller.getRightX()));
    controller
        .povLeft()
        .onTrue(new IntakeDeployZeroCommand(IntakeDeployZeroCommand.Direction.INWARD));
    controller
        .povUp()
        .onTrue(
            Commands.runOnce(() -> DashboardControls.adjustFlywheelPercent(FLYWHEEL_PERCENT_STEP)));
    controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> DashboardControls.adjustFlywheelPercent(-FLYWHEEL_PERCENT_STEP)));

    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      // Zero heading toward the opposing alliance wall. On red the field is
                      // flipped, so "forward" is 180 deg from the blue origin frame.
                      boolean isFlipped =
                          DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red;
                      drive.setPose(
                          new Pose2d(
                              drive.getPose().getTranslation(),
                              isFlipped ? Rotation2d.kPi : Rotation2d.kZero));
                    },
                    drive)
                .ignoringDisable(true));

    // controller
    //     .a()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               IntakeDeploy.mInstance.setPositionCentimeter(retract.getAsDouble(), 50, 1000,
    // 0);
    //               IntakeRoller.mInstance.setVelocityRotPerSec(40);
    //             },
    //             IntakeDeploy.mInstance,
    //             IntakeRoller.mInstance));
    // controller
    //     .a()
    //     .onFalse(
    //         new InstantCommand(
    //             () -> {
    //               IntakeDeploy.mInstance.setPositionCentimeter(retract.getAsDouble(), 50, 1000,
    // 0);
    //               IntakeRoller.mInstance.setV(0);
    //             },
    //             IntakeDeploy.mInstance,
    //             IntakeRoller.mInstance));
    controller.x().whileTrue(new Manual());
    controller
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  Indexer.mInstance.setV(-3);
                  Feeder.mInstance.setV(-3);
                }));
    controller
        .rightTrigger()
        .onFalse(
            new InstantCommand(
                () -> {
                  Indexer.mInstance.setV(0);
                  Feeder.mInstance.setV(0);
                }));

    controller
        .y()
        .onTrue(
            new HoodZeroCommand()
                .alongWith(new IntakeDeployZeroCommand(IntakeDeployZeroCommand.Direction.OUTWARD)));
    controller.rightBumper().whileTrue(new Shooting(false));
    controller.leftTrigger(0.1).whileTrue(new IntakeCommand());
    controller
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  Indexer.mInstance.setV(-3);
                  Feeder.mInstance.setV(-3);
                  IntakeRoller.mInstance.setV(-3);
                }));
    controller
        .leftBumper()
        .onFalse(
            new InstantCommand(
                () -> {
                  Indexer.mInstance.setV(0);
                  Feeder.mInstance.setV(0);
                  IntakeRoller.mInstance.setV(0);
                }));
    controller.a().whileTrue(new Shooting(true));

    // Sim-only: Start button clears every ball on the field and resets the robot pose / intake
    // stock. Ignored entirely on a real robot so the binding can stay in the code.
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (Constants.currentMode == Constants.Mode.SIM
                          && MapleSimArena.getInstance() != null) {
                        MapleSimArena.getInstance().resetField();
                      }
                    })
                .ignoringDisable(true));
  }

  /** Dashboard outputs refreshed every loop. */
  public void updateDashboardOutputs() {
    hood.publishDashboard();
  }
}
