package robotutils.perrobotconfig;

import java.util.Map;

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
        Map<int[], String> macToRobotNameDict,
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
