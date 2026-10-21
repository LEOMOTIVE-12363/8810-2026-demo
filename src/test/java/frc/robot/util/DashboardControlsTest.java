package frc.robot.util;

import static frc.robot.util.DashboardControlsTestUtil.DRIVE;
import static frc.robot.util.DashboardControlsTestUtil.FLYWHEEL;
import static frc.robot.util.DashboardControlsTestUtil.HOOD;
import static frc.robot.util.DashboardControlsTestUtil.TURN;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Hood.HoodConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class DashboardControlsTest {
  private static final double EPS = 1e-9;

  @AfterEach
  void resetDashboard() {
    DashboardControlsTestUtil.resetDefaults();
  }

  @Test
  void percentToScaleMapsPercentages() {
    assertEquals(0.0, DashboardControls.percentToScale(0.0), EPS);
    assertEquals(0.05, DashboardControls.percentToScale(5.0), EPS);
    assertEquals(0.5, DashboardControls.percentToScale(50.0), EPS);
    assertEquals(1.0, DashboardControls.percentToScale(100.0), EPS);
  }

  @Test
  void percentToScaleClampsAndDefaultsInvalidValues() {
    assertEquals(0.0, DashboardControls.percentToScale(-20.0), EPS);
    assertEquals(1.0, DashboardControls.percentToScale(250.0), EPS);
    assertEquals(1.0, DashboardControls.percentToScale(Double.NaN), EPS);
    assertEquals(1.0, DashboardControls.percentToScale(Double.POSITIVE_INFINITY), EPS);
  }

  @Test
  void hoodPercentMapsLinearlyOntoHoodRange() {
    double min = HoodConstants.MinAngleDeg;
    double max = HoodConstants.MaxAngleDeg;
    assertEquals(min, DashboardControls.hoodPercentToDeg(0.0), EPS);
    assertEquals(min + 0.05 * (max - min), DashboardControls.hoodPercentToDeg(5.0), EPS);
    assertEquals((min + max) / 2.0, DashboardControls.hoodPercentToDeg(50.0), EPS);
    assertEquals(max, DashboardControls.hoodPercentToDeg(100.0), EPS);
  }

  @Test
  void hoodPercentNeverLeavesMechanicalRange() {
    assertEquals(HoodConstants.MinAngleDeg, DashboardControls.hoodPercentToDeg(-50.0), EPS);
    assertEquals(HoodConstants.MaxAngleDeg, DashboardControls.hoodPercentToDeg(400.0), EPS);
    assertEquals(
        DashboardControls.hoodPercentToDeg(DashboardControls.DEFAULT_HOOD_PERCENT),
        DashboardControls.hoodPercentToDeg(Double.NaN),
        EPS);
  }

  @Test
  void driveAndTurnScalesAffectOnlyTheirOwnAxes() {
    ChassisSpeeds in = new ChassisSpeeds(4.0, -2.0, 6.0);
    for (double pct : new double[] {0.0, 5.0, 50.0, 100.0}) {
      double s = DashboardControls.percentToScale(pct);
      ChassisSpeeds drive = DashboardControls.scaleChassisSpeeds(in, s, 1.0);
      assertEquals(4.0 * s, drive.vxMetersPerSecond, EPS);
      assertEquals(-2.0 * s, drive.vyMetersPerSecond, EPS);
      assertEquals(6.0, drive.omegaRadiansPerSecond, EPS);
      ChassisSpeeds turn = DashboardControls.scaleChassisSpeeds(in, 1.0, s);
      assertEquals(4.0, turn.vxMetersPerSecond, EPS);
      assertEquals(-2.0, turn.vyMetersPerSecond, EPS);
      assertEquals(6.0 * s, turn.omegaRadiansPerSecond, EPS);
    }
  }

  @Test
  void defaultsLeaveCommandsUnchanged() {
    assertEquals(1.0, DashboardControls.getDriveScale(), EPS);
    assertEquals(1.0, DashboardControls.getTurnScale(), EPS);
    assertEquals(1.0, DashboardControls.getFlywheelScale(), EPS);
    assertEquals(50.0, DashboardControls.applyFlywheelLimit(50.0), EPS);
    ChassisSpeeds out = DashboardControls.applyDriveLimits(new ChassisSpeeds(1.0, 2.0, 3.0));
    assertEquals(1.0, out.vxMetersPerSecond, EPS);
    assertEquals(2.0, out.vyMetersPerSecond, EPS);
    assertEquals(3.0, out.omegaRadiansPerSecond, EPS);
  }

  @Test
  void dashboardEditsTakeEffectAtRuntimeAndStayIndependent() {
    DashboardControlsTestUtil.set(DRIVE, 5.0);
    assertEquals(0.05, DashboardControls.getDriveScale(), EPS);
    assertEquals(1.0, DashboardControls.getTurnScale(), EPS);
    assertEquals(1.0, DashboardControls.getFlywheelScale(), EPS);

    DashboardControlsTestUtil.set(TURN, 50.0);
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    DashboardControlsTestUtil.set(HOOD, 100.0);
    ChassisSpeeds out = DashboardControls.applyDriveLimits(new ChassisSpeeds(2.0, 4.0, 6.0));
    assertEquals(0.1, out.vxMetersPerSecond, EPS);
    assertEquals(0.2, out.vyMetersPerSecond, EPS);
    assertEquals(3.0, out.omegaRadiansPerSecond, EPS);
    assertEquals(0.0, DashboardControls.applyFlywheelLimit(60.0), EPS);
    assertEquals(HoodConstants.MaxAngleDeg, DashboardControls.getHoodTargetDeg(), EPS);

    // Out-of-range dashboard values are clamped, not passed through.
    DashboardControlsTestUtil.set(FLYWHEEL, 140.0);
    assertEquals(1.0, DashboardControls.getFlywheelScale(), EPS);
  }

  @Test
  void flywheelNudgeIsClamped() {
    DashboardControlsTestUtil.set(FLYWHEEL, 99.5);
    DashboardControls.adjustFlywheelPercent(1.0);
    DashboardControlsTestUtil.refresh();
    assertEquals(1.0, DashboardControls.getFlywheelScale(), EPS);

    DashboardControlsTestUtil.set(FLYWHEEL, 50.0);
    DashboardControls.adjustFlywheelPercent(-1.0);
    DashboardControlsTestUtil.refresh();
    assertEquals(0.49, DashboardControls.getFlywheelScale(), EPS);
  }
}
