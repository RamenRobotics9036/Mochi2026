package robotutils.perrobotconfig;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;
import robotutils.interfaces.MacKey;


/**
 * Takes a list of robot config objects.  When requested, returns the correct config
 * based on the Roborio MAC address.
 */
public class PerRobotConfig<T> {

    private String m_robotName = null;
    private String m_selectedConfigName = null;
    private T m_selectedConfig = null;
    private MacKey m_testMacKey = null;

    /** Constructor. */
    public PerRobotConfig(
        Map<MacKey, String> macToRobotNameDict,
        Map<String, String> robotNameToConfigNameDict,
        Map<String, T> configNameToConfigObjDict,
        String defaultConfigName,
        String simulationConfigName) {
        this(
            macToRobotNameDict,
            robotNameToConfigNameDict,
            configNameToConfigObjDict,
            defaultConfigName,
            simulationConfigName,
            null);
    }

    /**
     * Constructor for testing: treats testMacKey as the RoboRIO MAC address
     * instead of reading hardware.
     */
    public PerRobotConfig(
        Map<MacKey, String> macToRobotNameDict,
        Map<String, String> robotNameToConfigNameDict,
        Map<String, T> configNameToConfigObjDict,
        String defaultConfigName,
        String simulationConfigName,
        MacKey testMacKey) {

        m_testMacKey = testMacKey;

        validateInputMappings(
            macToRobotNameDict,
            robotNameToConfigNameDict,
            configNameToConfigObjDict,
            defaultConfigName,
            simulationConfigName);

        String robotName = identifyRobot(macToRobotNameDict);
        String configName = selectConfigName(
            robotName,
            robotNameToConfigNameDict,
            defaultConfigName,
            simulationConfigName);
        T config = getConfig(configName, configNameToConfigObjDict);

        // Set the results into member variables to save
        m_robotName = getRobotDisplayname(robotName);
        m_selectedConfigName = configName;
        m_selectedConfig = config;
    }

    /** Returns robot name. */
    public String getRobotName() {
        return m_robotName;
    }

    /** Returns the config for the current robot. */
    public T getBotConfig() {
        return m_selectedConfig;
    }

    /** Returns the name of the config for the current robot. */
    public String getBotConfigName() {
        return m_selectedConfigName;
    }

    /**
     * Validates that robot and config-name mappings are internally consistent.
     *
     * <p>String checks are case-sensitive.
     */
    private void validateInputMappings(
        Map<MacKey, String> macToRobotNameDict,
        Map<String, String> robotNameToConfigNameDict,
        Map<String, T> configNameToConfigObjDict,
        String defaultConfigName,
        String simulationConfigName) {

        if (macToRobotNameDict == null || macToRobotNameDict.isEmpty()) {
            throw new IllegalArgumentException(
                "macToRobotNameDict must contain at least one entry");
        }
        if (robotNameToConfigNameDict == null || robotNameToConfigNameDict.isEmpty()) {
            throw new IllegalArgumentException(
                "robotNameToConfigNameDict must contain at least one entry");
        }
        if (configNameToConfigObjDict == null || configNameToConfigObjDict.isEmpty()) {
            throw new IllegalArgumentException(
                "configNameToConfigObjDict must contain at least one entry");
        }
        if (defaultConfigName == null
            || !configNameToConfigObjDict.containsKey(defaultConfigName)) {

            throw new IllegalArgumentException(
                "defaultConfigName must be a key in configNameToConfigObjDict");
        }
        if (simulationConfigName == null
            || !configNameToConfigObjDict.containsKey(simulationConfigName)) {

            throw new IllegalArgumentException(
                "simulationConfigName must be a key in configNameToConfigObjDict");
        }

        for (String robotName : macToRobotNameDict.values()) {
            if (!robotNameToConfigNameDict.containsKey(robotName)) {
                throw new IllegalArgumentException(
                    "Missing robot->config mapping for robot name: " + robotName);
            }
        }

        for (String configName : robotNameToConfigNameDict.values()) {
            if (!configNameToConfigObjDict.containsKey(configName)) {
                throw new IllegalArgumentException(
                    "Missing config object for config name: " + configName);
            }
        }
    }

    private String identifyRobot(Map<MacKey, String> macToRobotNameDict) {
        for (Map.Entry<MacKey, String> entry : macToRobotNameDict.entrySet()) {
            boolean matches = (m_testMacKey != null)
                ? entry.getKey().equals(m_testMacKey)
                : MacAddress.isRobot(entry.getKey().suffixBytes());
            if (matches) {
                return entry.getValue();
            }
        }
        return null;
    }

    /** Given robot name or null, figures out whether to return robot
     * specific config, default config, or simulation config.
     */
    private String selectConfigName(
        String robotName,
        Map<String, String> robotNameToConfigNameDict,
        String defaultConfigName,
        String simulationConfigName) {

        if (RobotBase.isSimulation()) {
            return simulationConfigName;
        }
        if (robotName == null) {
            return defaultConfigName;
        }

        if (!robotNameToConfigNameDict.containsKey(robotName)) {
            throw new IllegalArgumentException(
                "No config mapping found for robot name: " + robotName);
        }

        return robotNameToConfigNameDict.get(robotName);
    }

    private T getConfig(
        String configName,
        Map<String, T> configNameToConfigObjDict) {

        if (!configNameToConfigObjDict.containsKey(configName)) {
            throw new IllegalArgumentException(
                "No config object found for config name: " + configName);
        }
        return configNameToConfigObjDict.get(configName);
    }

    private String getRobotDisplayname(String robotName) {
        if (RobotBase.isSimulation()) {
            return "Simulation";
        }
        if (robotName == null) {
            return "Unknown Robot";
        }
        return robotName;
    }
}
