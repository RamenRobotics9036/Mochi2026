package robotutils.perrobotconfig;

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

    /** Constructor. */
    public PerRobotConfig(
        Map<MacKey, String> macToRobotNameDict,
        Map<String, String> robotNameToConfigNameDict,
        Map<String, T> configNameToConfigObjDict,
        T defaultConfigName,
        T simulationConfigName) {

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

        validateInputMappings(
            macToRobotNameDict,
            robotNameToConfigNameDict,
            configNameToConfigObjDict);

        // $TODO4 - For now, just return the first
        m_robotName = macToRobotNameDict.values().iterator().next();
        m_selectedConfigName = robotNameToConfigNameDict.get(m_robotName);
        m_selectedConfig = configNameToConfigObjDict.get(m_selectedConfigName);
    }

    /**
     * Validates that robot and config-name mappings are internally consistent.
     *
     * <p>String checks are case-sensitive.
     */
    private void validateInputMappings(
        Map<MacKey, String> macToRobotNameDict,
        Map<String, String> robotNameToConfigNameDict,
        Map<String, T> configNameToConfigObjDict) {

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
}
