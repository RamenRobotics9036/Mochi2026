package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/** Helpers for joystick/controller input interpretation. */
public final class JoyHelpers {
    private JoyHelpers() {}

    /**
     * Returns {@code true} when {@code controller}'s D-pad is in
     * the downward region (135°–225°).
     */
    public static boolean isPovDownward(CommandXboxController controller) {
        int pov = controller.getHID().getPOV();
        return pov != -1 && (pov >= 135 && pov <= 225);
    }

    /**
     * Returns {@code true} when {@code controller}'s D-pad is in
     * the upward region (315°–360° or 0°–45°).
     */
    public static boolean isPovUpward(CommandXboxController controller) {
        int pov = controller.getHID().getPOV();
        return pov != -1 && (pov <= 45 || pov >= 315);
    }
}