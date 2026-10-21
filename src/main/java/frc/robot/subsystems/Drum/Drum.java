package frc.robot.subsystems.Drum;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOPhoenix6;
import frc.lib.io.MotorIOSim;
import frc.robot.Constants;
import frc.robot.util.DashboardControls;

public class Drum extends MotorSubsystem {
  public static final Drum mInstance = new Drum();

  private Drum() {
    super(
        switch (Constants.currentMode) {
          case REAL -> new MotorIOPhoenix6(DrumConstants.Drum_CONFIG);
          case SIM -> new MotorIOSim(DrumConstants.Drum_SIM_CONFIG);
          case REPLAY -> new MotorIO() {};
        },
        "Drum",
        "Disconnected Drum motor.");
  }

  /** Runs closed-loop velocity at the requested setpoint scaled by the Flywheel Speed Limit. */
  @Override
  public void setVelocityRotPerSec(double velocityRotPerSec) {
    super.setVelocityRotPerSec(DashboardControls.applyFlywheelLimit(velocityRotPerSec));
  }

  /**
   * True when the drum is within tolerance of the speed-limited version of a requested setpoint.
   * Never true while the limit holds the drum at zero, so fuel is not fed into a stopped drum.
   */
  public boolean isAtVelocity(double requestedRotPerSec, double toleranceRotPerSec) {
    if (DashboardControls.getFlywheelScale() <= 0.0) {
      return false;
    }
    double target = DashboardControls.applyFlywheelLimit(requestedRotPerSec);
    return Math.abs(getVelocityRotPerSec() - target) < toleranceRotPerSec;
  }
}
