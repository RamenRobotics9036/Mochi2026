package robotutils.perrobotconfig;

import java.util.Map;

import edu.wpi.first.wpilibj.RobotBase;
import robotutils.interfaces.MacKey;


/**
 * Takes a list of robot config objects.  When requested, returns the correct config
 * based on the Roborio MAC address.
 */
public class PerRobotConfig<T> {

    private String m_robotName = null;
    private String m_selectedConfigName = null;
    private T m_selectedConfig = null;

    /** Constructor. */
    public PerRobotConfig(
        Map<MacKey, String> macToRobotNameDict,
        Map<String, String> robotNameToConfigNameDict,
        Map<String, T> configNameToConfigObjDict,
        String defaultConfigName,
        String simulationConfigName) {

        validateInputMappings(
            macToRobotNameDict,
            robotNameToConfigNameDict,
            configNameToConfigObjDict,
            defaultConfigName,
            simulationConfigName);

        m_robotName = identifyRobot(macToRobotNameDict);

        // $TODO4 - For now, just return the first
        m_robotName = macToRobotNameDict.values().iterator().next();
        m_selectedConfigName = robotNameToConfigNameDict.get(m_robotName);
        m_selectedConfig = configNameToConfigObjDict.get(m_selectedConfigName);
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
            if (MacAddress.isRobot(entry.getKey().suffixBytes())) {
                return entry.getValue();
            }
        }
        return null;
    }
}
