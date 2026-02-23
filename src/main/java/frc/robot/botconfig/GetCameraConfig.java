package frc.robot.botconfig;

import frc.robot.Robot;
import frc.robot.Constants.SimVisionConstants;
import java.util.Map;

/** Helper class to get information on a specific camera.
 */
public class GetCameraConfig {
    BotConfigInterface m_configInterface;
    int m_totalCameraCount;

    private static final Map<Integer, String> m_simNamesDict = Map.of(
        0, SimVisionConstants.kLimelightNameSim,
        1, SimVisionConstants.kLimelightNameSim2
    );

    private final Map<Integer, String> m_physicalNamesDict;

    /** Constructor. */
    public GetCameraConfig(BotConfigInterface configInterface) {
        m_configInterface = configInterface;

        m_physicalNamesDict = Map.of(
            0, m_configInterface.getVisionLimelightNameReal()
        );

        m_totalCameraCount = Robot.isSimulation()
            ? m_simNamesDict.size()
            : m_physicalNamesDict.size();
    }

    /** Get number of cameras. */
    public int getTotalCameraCount() {
        return m_totalCameraCount;
    }

    /** Get name of a particular camera. */
    public String getCameraName(int cameraNum) {
        if (cameraNum < 0 || cameraNum >= m_totalCameraCount) {
            throw new IllegalArgumentException("Invalid camera number: " + cameraNum);
        }

        if (Robot.isSimulation()) {
            return m_simNamesDict.get(cameraNum);
        }
        else {
            return m_physicalNamesDict.get(cameraNum);
        }
    }
}
