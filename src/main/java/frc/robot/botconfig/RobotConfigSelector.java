package frc.robot.botconfig;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Map;
import robotutils.RobotUtilsFactory;
import robotutils.interfaces.MacKey;
import robotutils.interfaces.PerRobotConfigInterface;


/** Builds the per-robot config selector for this project. */
public final class RobotConfigSelector {
    private static final String COMPETITION_ROBOT_NAME = "Competition";
    private static final String PANCAKE_ROBOT_NAME = "Pancake";

    private RobotConfigSelector() {
    }

    /** Creates the selector used by the main robot project. */
    public static PerRobotConfigInterface<BotConfigInterface> create() {
        BotConfigInterface compConfig = new CompConfig();
        BotConfigInterface pancakeConfig = new PancakeConfig();
        String competitionConfigName = compConfig.getConfigName();
        String pancakeConfigName = pancakeConfig.getConfigName();

        PerRobotConfigInterface<BotConfigInterface> perRobotConfig = new RobotUtilsFactory().createPerRobotConfig(
            Map.of(
                new MacKey(0x38, 0xD2, 0x58), COMPETITION_ROBOT_NAME,
                new MacKey(0x38, 0xD9, 0x80), PANCAKE_ROBOT_NAME),
            Map.of(
                COMPETITION_ROBOT_NAME, competitionConfigName,
                PANCAKE_ROBOT_NAME, pancakeConfigName),
            Map.of(
                competitionConfigName, compConfig,
                pancakeConfigName, pancakeConfig),
            competitionConfigName,
            competitionConfigName);

        reportSelection(perRobotConfig);
        return perRobotConfig;
    }

    private static void reportSelection(PerRobotConfigInterface<BotConfigInterface> perRobotConfig) {
        String robotName = perRobotConfig.getRobotName();

        if ("Simulation".equals(robotName)) {
            System.out.println(">>> Detected: SIMULATION");
        }
        else if ("Unknown Robot".equals(robotName)) {
            DriverStation.reportError("UNKNOWN RIO MAC! Defaulting to COMPETITION.", false);
        }
        else {
            System.out.println(">>> Detected: " + robotName.toUpperCase());
        }
    }
}
