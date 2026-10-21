package frc.robot.subsystems.Hood;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOPhoenix6;
import frc.lib.io.MotorIOSim;
import frc.robot.Constants;

public class Hood extends MotorSubsystem {
  public static final Hood mInstance = new Hood();

  private Hood() {
    super(
        switch (Constants.currentMode) {
          case REAL -> new MotorIOPhoenix6(HoodConstants.Hood_CONFIG);
          case SIM -> new MotorIOSim(HoodConstants.Hood_SIM_CONFIG);
          case REPLAY -> new MotorIO() {};
        },
        "Hood",
        "Disconnected Hood motor.",
        true);
  }

  /** Measured hood angle in mechanism degrees, from the motor encoder (relative to the zero). */
  public double getAngleDeg() {
    return getPositionRot() * 360.0;
  }

  /** Measured hood angle as a percentage of the [MinAngleDeg, MaxAngleDeg] range (unclamped). */
  public double getAnglePercent() {
    return (getAngleDeg() - HoodConstants.MinAngleDeg)
        / (HoodConstants.MaxAngleDeg - HoodConstants.MinAngleDeg)
        * 100.0;
  }

  /** Publishes the measured hood angle to Elastic. */
  public void publishDashboard() {
    SmartDashboard.putNumber("Hood Actual Angle (%)", getAnglePercent());
    SmartDashboard.putNumber("Hood Actual Angle (deg)", getAngleDeg());
  }
}
