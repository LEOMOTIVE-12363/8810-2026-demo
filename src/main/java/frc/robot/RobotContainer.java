package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.BallFlowTracker;
import frc.robot.util.DashboardControls;
import frc.robot.util.DriverControls;
import frc.robot.util.DriverControls.Action;
import frc.robot.util.OperatorStatus;
import org.littletonrobotics.junction.Logger;

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
  private final LED led = LED.mInstance;

  // Port 1 is the preferred demo controller; port 0 retains the full original controls.
  private final DriverControls controls = new DriverControls();

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

  /** Bind the five demo actions on both ports, and the remaining actions on port 0 only. */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controls.leftY() * 0.9,
            () -> -controls.leftX() * 0.9,
            () -> -controls.rightX()));

    controls.onPress(
        Action.RETRACT,
        () -> new IntakeDeployZeroCommand(IntakeDeployZeroCommand.Direction.INWARD));
    controls.onPress(Action.RESET_HEADING, this::resetHeadingCommand);
    controls.whileHeld(Action.INTAKE, IntakeCommand::new);
    controls.whileHeld(Action.SHOOT, () -> new Shooting(false));
    controls.onPress(Action.EJECT, () -> reverseFeedPressCommand(true));
    controls.onRelease(Action.EJECT, () -> reverseFeedReleaseCommand(true));
    controls.whileHeld(Action.EJECT, () -> reverseFeedHeartbeatCommand(true));

    controls.onPress(
        Action.FLYWHEEL_UP,
        () ->
            Commands.runOnce(() -> DashboardControls.adjustFlywheelPercent(FLYWHEEL_PERCENT_STEP)));
    controls.onPress(
        Action.FLYWHEEL_DOWN,
        () ->
            Commands.runOnce(
                () -> DashboardControls.adjustFlywheelPercent(-FLYWHEEL_PERCENT_STEP)));
    controls.whileHeld(Action.MANUAL_SHOOT, Manual::new);
    controls.whileHeld(Action.CURRENT_FEED_SHOOT, () -> new Shooting(true));
    controls.onPress(Action.REVERSE_FEED, () -> reverseFeedPressCommand(false));
    controls.onRelease(Action.REVERSE_FEED, () -> reverseFeedReleaseCommand(false));
    controls.whileHeld(Action.REVERSE_FEED, () -> reverseFeedHeartbeatCommand(false));
    controls.onPress(
        Action.ZERO_OUTWARD,
        () ->
            new HoodZeroCommand()
                .alongWith(new IntakeDeployZeroCommand(IntakeDeployZeroCommand.Direction.OUTWARD)));
    controls.onPress(
        Action.RESET_SIM,
        () ->
            Commands.runOnce(
                    () -> {
                      if (Constants.currentMode == Constants.Mode.SIM
                          && MapleSimArena.getInstance() != null) {
                        MapleSimArena.getInstance().resetField();
                      }
                    })
                .ignoringDisable(true));
  }

  private Command resetHeadingCommand() {
    return Commands.runOnce(
            () -> {
              boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
              drive.setPose(
                  new Pose2d(
                      drive.getPose().getTranslation(),
                      isFlipped ? Rotation2d.kPi : Rotation2d.kZero));
            },
            drive)
        .ignoringDisable(true);
  }

  // Momentary reverse semantics, restored verbatim from the original pre-DriverControls bindings:
  // press/release write the motors with NO subsystem ownership, so a concurrently running Shooting
  // loop overwrites the reverse writes — that documented conflict is intentionally preserved
  // (2026-10-22 decision). Release, a disconnect, or losing controller priority still zeroes the
  // motors via the release binding, and the heartbeat drives only the display layers.
  private Command reverseFeedPressCommand(boolean includeIntake) {
    return Commands.runOnce(
        () -> {
          Indexer.mInstance.setV(-3);
          Feeder.mInstance.setV(-3);
          if (includeIntake) {
            IntakeRoller.mInstance.setV(-3);
          }
        });
  }

  private Command reverseFeedReleaseCommand(boolean includeIntake) {
    return Commands.runOnce(
        () -> {
          Indexer.mInstance.setV(0);
          Feeder.mInstance.setV(0);
          if (includeIntake) {
            IntakeRoller.mInstance.setV(0);
          }
        });
  }

  private Command reverseFeedHeartbeatCommand(boolean includeIntake) {
    return Commands.run(
        () -> {
          BallFlowTracker.report(
              includeIntake
                  ? BallFlowTracker.FlowKind.FULL_REVERSE
                  : BallFlowTracker.FlowKind.LOCAL_REVERSE);
          OperatorStatus.report(
              includeIntake
                  ? OperatorStatus.Action.REVERSING_FULL
                  : OperatorStatus.Action.REVERSING_LOCAL);
        });
  }

  /**
   * Dashboard outputs refreshed every loop. The operator-status panel is published separately by
   * {@link Robot}'s 10 Hz block so it stays live no matter which controller-binding scheme this
   * class carries.
   */
  public void updateDashboardOutputs() {
    hood.publishDashboard();
    Logger.recordOutput("Controls/ActiveController", controls.activePort());
    if (!Logger.hasReplaySource()) {
      SmartDashboard.putBoolean(
          "Controller 0 Connected", DriverStation.isJoystickConnected(DriverControls.FULL_PORT));
      SmartDashboard.putNumber("Active Controller", controls.activePort());
    }
  }
}
