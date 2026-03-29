package robotutils.perrobotconfig;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import robotutils.pub.interfaces.dashboard.DashboardProviderInterface;
import robotutils.pub.interfaces.dashboard.StringPublisherWrapper;

/** Dashboard provider for per-robot configuration values. */
public class PerRobotConfigDashboardProvider implements DashboardProviderInterface {
    private boolean m_isInitialized = false;
    private boolean m_isUpdated = false;
    private StringPublisherWrapper m_robotNamePublisher;
    private StringPublisherWrapper m_botConfigNamePublisher;

    /** Constructor. */
    public PerRobotConfigDashboardProvider() {
    }

    @Override
    public void init() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable tableRoot = inst.getTable(
                DashboardProviderInterface.getNetworkTableRoot());

        // Publish all the values to NetworkTables
        m_robotNamePublisher = new StringPublisherWrapper(
            tableRoot.getStringTopic("RobotName").publish());
        m_botConfigNamePublisher = new StringPublisherWrapper(
            tableRoot.getStringTopic("BotConfigName").publish());

        m_isInitialized = true;
    }

    @Override
    public void update() {
        if (!m_isInitialized) {
            throw new IllegalStateException("PerRobotConfigDashboardProvider not initialized");
        }

        // It's OK if they haven't yet updated the values, we just skip
        if (!m_isUpdated) {
            return;
        }

        m_robotNamePublisher

    }

    /** Caller should use this when new settings are available. */
    public void setLatestSettings(PerRobotConfigDashboardSettings settings) {


        m_isUpdated = true;
    }
}
