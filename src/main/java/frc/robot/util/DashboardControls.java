package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Hood.HoodConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Operator controls set from Elastic as percentages (0-100) under /SmartDashboard.
 *
 * <p>Drive and Turn scale the chassis translational and rotational velocity independently inside
 * {@link frc.robot.subsystems.drive.Drive}. They are not applied during autonomous so auto paths
 * run at their tuned speeds.
 *
 * <p>Flywheel scales the drum velocity setpoint inside {@link frc.robot.subsystems.Drum.Drum} (so
 * the closed-loop velocity control is unchanged), and Hood Angle maps linearly onto the hood's
 * [{@link HoodConstants#MinAngleDeg}, {@link HoodConstants#MaxAngleDeg}] range. Together they are
 * the shot configuration used by {@link frc.robot.commands.ShootingCommand.Shooting}, in every
 * mode.
 *
 * <p>The entries are {@link LoggedNetworkNumber}s: the default is only set if the topic has no
 * value (so dashboard edits are never overwritten), and every value read is logged for replay.
 */
public final class DashboardControls {
  public static final double DEFAULT_SPEED_PERCENT = 100.0;
  public static final double DEFAULT_HOOD_PERCENT = 50.0;

  private static final LoggedNetworkNumber drivePercent =
      new LoggedNetworkNumber("/SmartDashboard/Drive Speed Limit (%)", DEFAULT_SPEED_PERCENT);
  private static final LoggedNetworkNumber turnPercent =
      new LoggedNetworkNumber("/SmartDashboard/Turn Speed Limit (%)", DEFAULT_SPEED_PERCENT);
  private static final LoggedNetworkNumber flywheelPercent =
      new LoggedNetworkNumber("/SmartDashboard/Flywheel Speed Limit (%)", DEFAULT_SPEED_PERCENT);
  private static final LoggedNetworkNumber hoodPercent =
      new LoggedNetworkNumber("/SmartDashboard/Hood Angle (%)", DEFAULT_HOOD_PERCENT);

  private DashboardControls() {}

  /** Clamps a dashboard percentage to [0, 100]; NaN/inf fall back to defaultPercent. */
  static double sanitizePercent(double percent, double defaultPercent) {
    if (!Double.isFinite(percent)) {
      percent = defaultPercent;
    }
    return MathUtil.clamp(percent, 0.0, 100.0);
  }

  /** Converts a dashboard speed percentage to a 0-1 scale: clamped, NaN/inf -> 100%. */
  static double percentToScale(double percent) {
    return sanitizePercent(percent, DEFAULT_SPEED_PERCENT) / 100.0;
  }

  /** Target Angle = Min + (Percentage / 100) * (Max - Min), with the percentage clamped. */
  static double hoodPercentToDeg(double percent) {
    double fraction = sanitizePercent(percent, DEFAULT_HOOD_PERCENT) / 100.0;
    return HoodConstants.MinAngleDeg
        + fraction * (HoodConstants.MaxAngleDeg - HoodConstants.MinAngleDeg);
  }

  /** Scales translation by driveScale and rotation by turnScale, independently. */
  static ChassisSpeeds scaleChassisSpeeds(
      ChassisSpeeds speeds, double driveScale, double turnScale) {
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond * driveScale,
        speeds.vyMetersPerSecond * driveScale,
        speeds.omegaRadiansPerSecond * turnScale);
  }

  private static double teleopScale(LoggedNetworkNumber percent) {
    return DriverStation.isAutonomous() ? 1.0 : percentToScale(percent.get());
  }

  /** Translational scale (0-1) currently applied to the drivetrain. */
  public static double getDriveScale() {
    return teleopScale(drivePercent);
  }

  /** Rotational scale (0-1) currently applied to the drivetrain. */
  public static double getTurnScale() {
    return teleopScale(turnPercent);
  }

  /** Scale (0-1) currently applied to the flywheel velocity setpoint. */
  public static double getFlywheelScale() {
    return percentToScale(flywheelPercent.get());
  }

  /** Hood target (mechanism degrees) from the Hood Angle (%) entry. */
  public static double getHoodTargetDeg() {
    return hoodPercentToDeg(hoodPercent.get());
  }

  /** Applies the Drive and Turn limits to a chassis speed command. */
  public static ChassisSpeeds applyDriveLimits(ChassisSpeeds speeds) {
    return scaleChassisSpeeds(speeds, getDriveScale(), getTurnScale());
  }

  /** Applies the Flywheel limit to a drum velocity setpoint. */
  public static double applyFlywheelLimit(double velocityRotPerSec) {
    return velocityRotPerSec * getFlywheelScale();
  }

  /** Nudges the Flywheel entry by deltaPercent (clamped); Elastic shows it on the next update. */
  public static void adjustFlywheelPercent(double deltaPercent) {
    flywheelPercent.set(
        sanitizePercent(flywheelPercent.get() + deltaPercent, DEFAULT_SPEED_PERCENT));
  }

  /** Logs the effective (clamped, currently applied) values for the dashboard and log review. */
  public static void logEffectiveValues() {
    Logger.recordOutput("DashboardControls/DriveEffectivePercent", getDriveScale() * 100.0);
    Logger.recordOutput("DashboardControls/TurnEffectivePercent", getTurnScale() * 100.0);
    Logger.recordOutput("DashboardControls/FlywheelEffectivePercent", getFlywheelScale() * 100.0);
    Logger.recordOutput("DashboardControls/HoodTargetDeg", getHoodTargetDeg());
  }
}
