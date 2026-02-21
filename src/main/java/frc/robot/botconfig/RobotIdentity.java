package frc.robot.botconfig;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.util.MACAddress;

/**
 * Automatically picks the right robot constants based on the RoboRIO's MAC address.
 */
public class RobotIdentity {

    /** Holds the detected robot's config and name together. */
    public static class IdentityResult {
        private final BotConfigInterface config;
        private final String name;

        public IdentityResult(BotConfigInterface config, String name) {
            this.config = config;
            this.name = name;
        }

        public BotConfigInterface getConfig() { return config; }
        public String getName() { return name; }
    }

    // To find a new MAC: Connect to robot, run 'arp -a' in terminal.
    // Look for 10.90.36.2 and copy the last 3 hex pairs.

    // Comp Bot RIO: 00-80-2F-34-4C-2D
    private static final int[] COMP_BOT_MAC = {0x34, 0x4C, 0x2D};

    // Pancake RIO: 00-80-2F-38-D9-80
    private static final int[] PANCAKE_MAC  = {0x38, 0xD9, 0x80};

    private static IdentityResult m_identityResult = null;

    /** Returns the config for the current robot. */
    public static BotConfigInterface getBotConfig() {
        return getIdentityResult().getConfig();
    }

    public static String getBotName() {
        return getIdentityResult().getName();
    }

    /** Returns the identity (config + name) for the current robot. */
    private static IdentityResult getIdentityResult() {
        if (m_identityResult == null) {
            detectRobot();
        }
        return m_identityResult;
    }

    private static void detectRobot() {
        if (MACAddress.isRobot(COMP_BOT_MAC)) {
            System.out.println(">>> Detected: COMPETITION ROBOT");
            m_identityResult = new IdentityResult(new CompConfig(), "Competition");
        }
        else if (MACAddress.isRobot(PANCAKE_MAC)) {
            System.out.println(">>> Detected: PANCAKE (PRACTICE)");
            m_identityResult = new IdentityResult(new PancakeConfig(), "Pancake");
        }
        else if (Robot.isSimulation()) {
            System.out.println(">>> Detected: SIMULATION");
            m_identityResult = new IdentityResult(new CompConfig(), "Simulation");
        }
        else {
            DriverStation.reportError("UNKNOWN RIO MAC! Defaulting to COMPETITION.", false);
            m_identityResult = new IdentityResult(new CompConfig(), "Unknown");
        }
    }
}
