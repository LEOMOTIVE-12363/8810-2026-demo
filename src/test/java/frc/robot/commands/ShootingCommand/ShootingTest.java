package frc.robot.commands.ShootingCommand;

import static frc.robot.util.DashboardControlsTestUtil.FLYWHEEL;
import static frc.robot.util.DashboardControlsTestUtil.HOOD;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Drum.DrumConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodConstants;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.DashboardControlsTestUtil;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Runs Shooting against the sim mechanism IO with no Drive, pose, or vision ever created: if the
 * command touched any of them it would throw on the null Drive.mInstance.
 */
class ShootingTest {
  private static final double EPS = 1e-6;

  @BeforeAll
  static void setUpHal() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
  }

  @AfterEach
  void resetDashboard() {
    DashboardControlsTestUtil.resetDefaults();
  }

  /** One robot loop: run the command, advance time, then update mechanism inputs. */
  private static void step(Shooting shooting) {
    shooting.execute();
    SimHooks.stepTiming(0.02);
    Drum.mInstance.periodic();
    Hood.mInstance.periodic();
    Feeder.mInstance.periodic();
    Indexer.mInstance.periodic();
    IntakeDeploy.mInstance.periodic();
    IntakeRoller.mInstance.periodic();
  }

  @Test
  void shootsFromDashboardSettingsWithoutPoseOrVision() {
    assertNull(Drive.mInstance, "test must run with no drivetrain / pose source");
    DashboardControlsTestUtil.set(FLYWHEEL, 50.0);
    DashboardControlsTestUtil.set(HOOD, 100.0);

    Shooting shooting = new Shooting(false);
    assertFalse(shooting.getRequirements().contains(Drive.mInstance));
    shooting.initialize();
    for (int i = 0; i < 20; i++) {
      step(shooting);
    }

    assertEquals(
        0.5 * DrumConstants.MaxShotVelocity, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(HoodConstants.MaxAngleDeg / 360.0, Hood.mInstance.getPositionSetpointRot(), EPS);
    // Reached SHOOT: the feeder is running at the tuned feed speed.
    assertEquals(
        ShootingConstants.FeederRotpsTunable.getAsDouble(),
        Feeder.mInstance.getVelocitySetpointRotPerSec(),
        EPS);

    // Dashboard edits mid-volley change the commanded setpoints on the next loop.
    DashboardControlsTestUtil.set(FLYWHEEL, 5.0);
    DashboardControlsTestUtil.set(HOOD, 0.0);
    step(shooting);
    assertEquals(
        0.05 * DrumConstants.MaxShotVelocity, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(HoodConstants.MinAngleDeg / 360.0, Hood.mInstance.getPositionSetpointRot(), EPS);

    shooting.end(true);
  }

  @Test
  void zeroFlywheelNeverFeeds() {
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);

    Shooting shooting = new Shooting(false);
    shooting.initialize();
    for (int i = 0; i < 50; i++) {
      step(shooting);
    }

    assertEquals(0.0, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS);
    shooting.end(true);
  }
}
