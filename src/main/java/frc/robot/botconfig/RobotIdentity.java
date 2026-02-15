package frc.robot.botconfig;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.MACAddress;

/**
 * Automatically picks the right robot constants based on the RoboRIO's MAC address.
 */
public class RobotIdentity {
    
    // To find a new MAC: Connect to robot, run 'arp -a' in terminal. 
    // Look for 10.90.36.2 and copy the last 3 hex pairs.

    // Comp Bot RIO: 00-80-2F-34-4C-2D
    private static final int[] COMP_BOT_MAC = {0x34, 0x4C, 0x2D};
    
    // Pancake RIO: 00-80-2F-38-D9-80
    private static final int[] PANCAKE_MAC  = {0x38, 0xD9, 0x80}; 

    private static BotConfigInterface m_config = null;

    /** * Returns the config for the current robot. 
     * Defaults to Competition if the hardware is unknown.
     */
    public static BotConfigInterface getMode() {
        if (m_config != null) return m_config; // Return cached config if already found

        if (MACAddress.isRobot(COMP_BOT_MAC)) {
            System.out.println(">>> Detected: COMPETITION ROBOT");
            m_config = new CompConfig();
        } 
        else if (MACAddress.isRobot(PANCAKE_MAC)) {
            System.out.println(">>> Detected: PANCAKE (PRACTICE)");
            m_config = new PancakeConfig();
        } 
        else {
            // Default to Comp so the robot is match-ready even on a spare RIO
            DriverStation.reportError("UNKNOWN RIO MAC! Defaulting to COMPETITION.", false);
            m_config = new CompConfig();
        }

        return m_config;
    }

    /** Helper to check if we are on the Comp bot */
    public static boolean isCompetition() {
        return getMode() instanceof CompConfig;
    }
}