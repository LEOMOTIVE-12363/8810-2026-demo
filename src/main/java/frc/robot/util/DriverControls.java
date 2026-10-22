package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Predicate;
import java.util.function.Supplier;
import java.util.function.ToDoubleFunction;

/** Exclusive controller routing: port 1 (demo) takes precedence over port 0 (full controls). */
public final class DriverControls {
  public static final int FULL_PORT = 0;
  public static final int DEMO_PORT = 1;

  /** The demo controller exposes only the five illustrated actions, plus the drive axes. */
  public enum Action {
    INTAKE(false, hid -> hid.getLeftTriggerAxis() > 0.1),
    EJECT(false, XboxController::getLeftBumperButton),
    SHOOT(false, XboxController::getRightBumperButton),
    RETRACT(false, hid -> hid.getPOV() == 270),
    RESET_HEADING(false, XboxController::getBButton),
    CURRENT_FEED_SHOOT(true, XboxController::getAButton),
    MANUAL_SHOOT(true, XboxController::getXButton),
    ZERO_OUTWARD(true, XboxController::getYButton),
    REVERSE_FEED(true, hid -> hid.getRightTriggerAxis() > 0.5),
    FLYWHEEL_UP(true, hid -> hid.getPOV() == 0),
    FLYWHEEL_DOWN(true, hid -> hid.getPOV() == 180),
    RESET_SIM(true, XboxController::getStartButton);

    private final boolean fullOnly;
    private final Predicate<XboxController> pressed;

    Action(boolean fullOnly, Predicate<XboxController> pressed) {
      this.fullOnly = fullOnly;
      this.pressed = pressed;
    }
  }

  private final XboxController[] controllers = {
    new XboxController(FULL_PORT), new XboxController(DEMO_PORT)
  };

  /** Returns -1 when neither controller is connected; priority is connection-based, not motion. */
  public int activePort() {
    if (DriverStation.isJoystickConnected(DEMO_PORT)) {
      return DEMO_PORT;
    }
    return DriverStation.isJoystickConnected(FULL_PORT) ? FULL_PORT : -1;
  }

  public double leftX() {
    return axis(XboxController::getLeftX);
  }

  public double leftY() {
    return axis(XboxController::getLeftY);
  }

  public double rightX() {
    return axis(XboxController::getRightX);
  }

  private double axis(ToDoubleFunction<XboxController> read) {
    int port = activePort();
    return port < 0 ? 0.0 : read.applyAsDouble(controllers[port]);
  }

  public void whileHeld(Action action, Supplier<Command> command) {
    bind(action, command, true);
  }

  public void onPress(Action action, Supplier<Command> command) {
    bind(action, command, false);
  }

  /**
   * Runs the command once when the hold ends — a normal release, a disconnect, or a priority loss
   * all make the gated trigger go false. Unlike {@link #onPress}/{@link #whileHeld} the command is
   * NOT wrapped in {@code onlyWhile}: the press gate has already gone false by the time this fires,
   * and the cleanup must run exactly then.
   */
  public void onRelease(Action action, Supplier<Command> command) {
    for (int port = FULL_PORT; port <= DEMO_PORT; port++) {
      if (action.fullOnly && port == DEMO_PORT) {
        continue;
      }
      final int source = port;
      Trigger trigger =
          new Trigger(() -> activePort() == source && action.pressed.test(controllers[source]));
      trigger.onFalse(command.get());
    }
  }

  private void bind(Action action, Supplier<Command> factory, boolean held) {
    for (int port = FULL_PORT; port <= DEMO_PORT; port++) {
      if (action.fullOnly && port == DEMO_PORT) {
        continue;
      }
      final int source = port;
      Trigger trigger =
          new Trigger(() -> activePort() == source && action.pressed.test(controllers[source]));
      // Separate command instances per source. In particular, a one-shot homing command must
      // also stop if its controller disconnects or loses priority before reaching the hard stop.
      Command command = factory.get().onlyWhile(() -> activePort() == source);
      if (held) {
        trigger.whileTrue(command);
      } else {
        trigger.onTrue(command);
      }
    }
  }
}
