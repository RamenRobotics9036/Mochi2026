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
        /** The config for the current robot. */
        private final BotConfigInterface config;
        /** The name of the current robot. */
        private final String name;

        /**
         * Create a new IdentityResult object to store information about the robot.
         *
         * @param config The config for the current robot
         * @param name The name of the current robot
         */
        public IdentityResult(BotConfigInterface config, String name) {
            this.config = config;
            this.name = name;
        }

        /** Returns the config for the current robot. */
        public BotConfigInterface getConfig() { return config; }
        /** Returns the name of the current robot. */
        public String getName() { return name; }
    }

    // To find a new MAC: Connect to robot, run 'arp -a' in terminal.
    // Look for 10.90.36.2 and copy the last 3 hex pairs.

    // Comp Bot RIO: 00-80-2F-38-D2-58
    private static final int[] COMP_BOT_MAC = {0x38, 0xD2, 0x58};

    // Pancake RIO: 00-80-2F-38-D9-80
    private static final int[] PANCAKE_MAC  = {0x38, 0xD9, 0x80};

    //** The identity of the current robot. null until getIdentityResult is run. */
    private static IdentityResult m_identityResult = null;

    /** Returns the config for the current robot. */
    public static BotConfigInterface getBotConfig() {
        return getIdentityResult().getConfig();
    }

    /** Returns the name of the current robot. */
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

    /** Uses the MAC address of the current robot to figure out which bot the code is running on. */
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
